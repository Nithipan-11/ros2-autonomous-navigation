#!/usr/bin/env python3
"""Pull the robot's driven route out of a ROS 2 bag (.db3) - no ROS install needed.

  python3 tools/course/extract_route.py course_run/course_run_0.db3 route.json

Writes rows of [t, lidar_x, lidar_y, axle_x, axle_y, yaw] (same format plot_course.py reads),
starting from when the robot first moves.
"""
import json
import math
import sqlite3
import struct
import sys

BASE_OFFSET = 1.3  # lidar is 1.3 m ahead of the rear axle


def parse_odometry(buf):
    """Minimal CDR decode of nav_msgs/Odometry up to pose.orientation."""
    body = buf[4:]  # skip 4-byte encapsulation header
    off = 0

    def align(n):
        nonlocal off
        off = (off + n - 1) // n * n

    def u32():
        nonlocal off
        align(4)
        v = struct.unpack_from("<I", body, off)[0]
        off += 4
        return v

    def string():
        nonlocal off
        n = u32()
        off += n

    def f64():
        nonlocal off
        align(8)
        v = struct.unpack_from("<d", body, off)[0]
        off += 8
        return v

    sec = u32()
    nsec = u32()
    string()  # header.frame_id
    string()  # child_frame_id
    x, y, _z = f64(), f64(), f64()
    qx, qy, qz, qw = f64(), f64(), f64(), f64()
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return sec + nsec * 1e-9, x, y, yaw


def main():
    db, out = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "route.json"
    con = sqlite3.connect(db)
    tid = con.execute("SELECT id FROM topics WHERE name = '/odom/filtered'").fetchone()
    if not tid:
        sys.exit("No /odom/filtered topic in this bag")
    rows = con.execute("SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp", tid).fetchall()

    pts = [parse_odometry(r[0]) for r in rows]
    # Start where the robot first moves, keep ~5 samples per second
    x0, y0 = pts[0][1], pts[0][2]
    start = next((i for i, p in enumerate(pts) if math.hypot(p[1] - x0, p[2] - y0) > 0.05), 0)
    t0 = pts[start][0]
    traj, last_t = [], -1.0
    for t, x, y, yaw in pts[max(start - 1, 0):]:
        if t - last_t < 0.2:
            continue
        last_t = t
        traj.append([round(t - t0, 2), round(x, 3), round(y, 3),
                     round(x - BASE_OFFSET * math.cos(yaw), 3), round(y - BASE_OFFSET * math.sin(yaw), 3),
                     round(yaw, 3)])
    json.dump(traj, open(out, "w"))
    dist = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(traj, traj[1:]))
    print(f"Wrote {len(traj)} points to {out}: {traj[-1][0]:.0f} s, {dist:.1f} m")


if __name__ == "__main__":
    main()
