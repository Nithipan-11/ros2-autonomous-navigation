#!/usr/bin/env python3
"""Draw the obstacle course (and optionally a driven trajectory) to a PNG.

  python3 tools/course/plot_course.py                      -> course.png (layout only)
  python3 tools/course/plot_course.py traj.json run.png    -> layout + driven path
traj.json rows: [t, lidar_x, lidar_y, axle_x, axle_y, yaw]
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

HERE = Path(__file__).parent
course = json.loads((HERE / "course.json").read_text())
traj = json.loads(Path(sys.argv[1]).read_text()) if len(sys.argv) > 1 else None
out = sys.argv[2] if len(sys.argv) > 2 else str(HERE / "course.png")

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_facecolor("#f4f4f2")
for b in course["boxes"]:
    wall = b["name"].startswith(("wall", "divider"))
    ax.add_patch(Rectangle((b["cx"] - b["sx"] / 2, b["cy"] - b["sy"] / 2), b["sx"], b["sy"],
                           color="#555555" if wall else "#2f6fde"))
for c in course["cylinders"]:
    ax.add_patch(Circle((c["cx"], c["cy"]), c["r"], color="#f08a24"))

s = course["spawn"]
ax.plot(s["x"] + 1.3, s["y"], marker="s", color="black", ms=9)
ax.annotate("START", (s["x"] + 1.3, s["y"]), xytext=(6, -14), textcoords="offset points", fontsize=9)
for i, (x, y) in enumerate(course["checkpoints"], 1):
    ax.plot(x, y, "o", color="#1aa260", ms=13)
    ax.annotate(str(i), (x, y), ha="center", va="center", color="white", fontsize=8, weight="bold")

if traj:
    xs = [r[1] for r in traj]
    ys = [r[2] for r in traj]
    ax.plot(xs, ys, color="#d6336c", lw=2, label=f"driven path ({traj[-1][0]:.0f} s)")
    ax.legend(loc="lower right")

ax.set_xlim(-15.5, 15.5)
ax.set_ylim(-15.5, 15.5)
ax.set_aspect("equal")
ax.set_title("Obstacle course: slalom -> pillar field -> gate + chicane")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
fig.tight_layout()
fig.savefig(out, dpi=120)
print("wrote", out)
