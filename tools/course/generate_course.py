#!/usr/bin/env python3
"""
Obstacle-course generator.

Single source of truth for the course layout. Writes:
  src/gazebo/launch/course_env.sdf   - Gazebo world (physics + lidar sees these)
  src/gazebo/launch/course_env.urdf  - same obstacles for Foxglove's URDF layer
  tools/course/course.json           - obstacles + checkpoints (used by the test sim)

Run from the repo root:  python3 tools/course/generate_course.py
"""
import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / "src" / "gazebo" / "launch"

H = 3.0  # obstacle height (m)

# ---------------------------------------------------------------------------
# Layout: 30 x 30 m arena split into three lanes the robot snakes through.
#
#   y=+15 +--------------------------------------------------+
#         | TOP LANE  (left -> right): gate, then chicane -> FINISH
#   y=+5  |      +==============================================|  divider 2
#         | MIDDLE LANE (right -> left): pillar field          |
#   y=-5  |==============================================+      |  divider 1
#         | BOTTOM LANE (left -> right): slalom baffles        |
#   y=-15 +--START-------------------------------------------+
# ---------------------------------------------------------------------------

# Boxes: (name, cx, cy, size_x, size_y)
boxes = [
    # Arena walls
    ("wall_s", 0, -15, 30, 0.5),
    ("wall_n", 0, 15, 30, 0.5),
    ("wall_e", 15, 0, 0.5, 30),
    ("wall_w", -15, 0, 0.5, 30),
    # Lane dividers (each leaves a 6 m opening at one end)
    ("divider1", -3, -5, 24, 0.5),   # x in [-15, 9]  -> opening on the right
    ("divider2", 3, 5, 24, 0.5),     # x in [-9, 15]  -> opening on the left
    # Bottom lane: slalom baffles, alternating from the floor wall and the divider
    ("baffle1", -6, -12.0, 0.5, 5.5),    # from bottom wall, 4 m gap above
    ("baffle2", 0, -8.0, 0.5, 5.5),      # from divider,     4 m gap below
    ("baffle3", 6, -12.0, 0.5, 5.5),     # from bottom wall, 4 m gap above
    # Top lane: after the gate, a block rising from divider2 forces the robot high (chicane)
    ("chicane", 6, 8.0, 2.0, 5.5),       # y in [5.25, 10.75] -> 4 m gap above
]

def gate(x, y_lo_wall, y_hi_wall, centre, width):
    """Two wall stubs at x leaving an opening `width` wide centred on `centre`."""
    lo_end = centre - width / 2
    hi_start = centre + width / 2
    lo = (x, (y_lo_wall + lo_end) / 2, 0.5, lo_end - y_lo_wall)
    hi = (x, (hi_start + y_hi_wall) / 2, 0.5, y_hi_wall - hi_start)
    return lo, hi

# Top-lane gate: inner faces of divider2 (5.25) and the north wall (14.75)
(lx, ly, lsx, lsy), (hx, hy, hsx, hsy) = gate(-2, 5.25, 14.75, centre=10.0, width=4.0)
boxes += [("gate_lo", lx, ly, lsx, lsy), ("gate_hi", hx, hy, hsx, hsy)]

# Cylinders: (name, cx, cy, radius) - middle-lane pillar field.
# Every opening is >= 4 m: the robot needs ~3.2 m (2 x 1.6 m circumscribed radius).
cylinders = [
    ("pillar1", 7.0, 0.0, 0.75),     # pass above or below
    ("pillar2", 1.0, 3.0, 0.75),     # pillar2 + pillar3: only the 4.5 m middle gap is wide enough
    ("pillar3", 1.0, -3.0, 0.75),
    ("pillar4", -5.0, 0.0, 0.75),    # pass above or below
]

# Robot spawn (wheel-axle pose): bottom-left, facing +x
spawn = (-13.0, -12.0, 0.0)

# Checkpoints the robot must pass, in order (lidar point targets).
# A* works out how to get between them; these just define the course order.
checkpoints = [
    (11.5, -10.5),   # end of the slalom lane
    (11.5, 0.0),     # through the right opening into the middle lane
    (-12.0, 0.0),    # through the pillar field
    (-12.0, 10.0),   # up the left opening into the top lane
    (12.0, 10.0),    # gate + chicane -> FINISH
]

# ---------------------------------------------------------------------------

def box_model(name, cx, cy, sx, sy, rgba):
    r, g, b = rgba
    return f"""
        <model name='{name}'>
            <static>true</static>
            <pose>{cx} {cy} 0 0 0 0</pose>
            <link name='link'>
                <visual name='visual'>
                    <geometry><box><size>{sx} {sy} {H}</size></box></geometry>
                    <material>
                        <ambient>{r} {g} {b} 1</ambient>
                        <diffuse>{r} {g} {b} 1</diffuse>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry><box><size>{sx} {sy} {H}</size></box></geometry>
                </collision>
            </link>
        </model>"""

def cyl_model(name, cx, cy, rad):
    return f"""
        <model name='{name}'>
            <static>true</static>
            <pose>{cx} {cy} 0 0 0 0</pose>
            <link name='link'>
                <visual name='visual'>
                    <geometry><cylinder><radius>{rad}</radius><length>{H}</length></cylinder></geometry>
                    <material>
                        <ambient>1.0 0.5 0.0 1</ambient>
                        <diffuse>1.0 0.5 0.0 1</diffuse>
                    </material>
                </visual>
                <collision name='collision'>
                    <geometry><cylinder><radius>{rad}</radius><length>{H}</length></cylinder></geometry>
                </collision>
            </link>
        </model>"""

def write_sdf():
    src = (LAUNCH / "robot_env.sdf").read_text()
    start = src.index("<model name='wall'>")
    end = src.index("<model name='robot'")
    models = []
    for name, cx, cy, sx, sy in boxes:
        colour = (0.4, 0.4, 0.4) if name.startswith(("wall", "divider")) else (0.0, 0.3, 1.0)
        models.append(box_model(name, cx, cy, sx, sy, colour))
    for name, cx, cy, r in cylinders:
        models.append(cyl_model(name, cx, cy, r))
    out = src[:start] + "".join(models).lstrip() + "\n\n        " + src[end:]
    sx, sy, syaw = spawn
    out = re.sub(r"(<model name='robot' canonical_link='chassis'>\s*<pose relative_to='world'>)[^<]*(</pose>)",
                 rf"\g<1>{sx} {sy} 1 0 0 {syaw}\g<2>", out, count=1)
    (LAUNCH / "course_env.sdf").write_text(out)

def write_urdf():
    parts = ['<?xml version="1.0" ?>', "<robot name='course'>", '<link name="sim_world"></link>']
    for name, cx, cy, sx, sy in boxes:
        parts.append(f"""
<joint name="{name}_joint" type="fixed"><origin xyz="{cx} {cy} {H/2}"/><parent link="sim_world"/><child link="{name}"/></joint>
<link name="{name}"><visual><geometry><box size="{sx} {sy} {H}"/></geometry></visual></link>""")
    for name, cx, cy, r in cylinders:
        parts.append(f"""
<joint name="{name}_joint" type="fixed"><origin xyz="{cx} {cy} {H/2}"/><parent link="sim_world"/><child link="{name}"/></joint>
<link name="{name}"><visual><geometry><cylinder radius="{r}" length="{H}"/></geometry></visual></link>""")
    parts.append("</robot>\n")
    (LAUNCH / "course_env.urdf").write_text("\n".join(parts))

def write_json():
    data = {
        "boxes": [dict(name=n, cx=cx, cy=cy, sx=sx, sy=sy) for n, cx, cy, sx, sy in boxes],
        "cylinders": [dict(name=n, cx=cx, cy=cy, r=r) for n, cx, cy, r in cylinders],
        "spawn": dict(x=spawn[0], y=spawn[1], yaw=spawn[2]),
        "checkpoints": [list(c) for c in checkpoints],
    }
    (Path(__file__).parent / "course.json").write_text(json.dumps(data, indent=2))

if __name__ == "__main__":
    write_sdf()
    write_urdf()
    write_json()
    flat = ", ".join(f"{x}, {y}" for x, y in checkpoints)
    print(f"Wrote course_env.sdf, course_env.urdf, course.json ({len(boxes)} boxes, {len(cylinders)} pillars)")
    print(f"planner waypoints: [{flat}]")
