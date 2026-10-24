#!/usr/bin/env python3
"""Dark, portfolio-styled course + route images.

  python3 tools/course/plot_route_dark.py route.json out_dir/
Writes out_dir/nav-route.png (square) and out_dir/nav-thumbnail.png (16:9 card image).
"""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

BG, SURFACE, BORDER, BRIGHT = "#0a0e13", "#10161d", "#1f2932", "#2d3a44"
ACCENT, FG, MUTED = "#4ade80", "#e7ebee", "#8b98a3"
MONO = "DejaVu Sans Mono"

HERE = Path(__file__).parent
course = json.loads((HERE / "course.json").read_text())
traj = json.loads(Path(sys.argv[1]).read_text())
out = Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
dist = sum(((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5 for a, b in zip(traj, traj[1:]))


def draw_course(ax):
    ax.set_facecolor(SURFACE)
    # faint 1 m grid, like a PCB/grid paper
    for v in range(-15, 16, 1):
        a = 0.35 if v % 5 else 0.7
        ax.axhline(v, color=BORDER, lw=0.6, alpha=a, zorder=0)
        ax.axvline(v, color=BORDER, lw=0.6, alpha=a, zorder=0)
    for b in course["boxes"]:
        wall = b["name"].startswith(("wall", "divider"))
        ax.add_patch(Rectangle((b["cx"] - b["sx"] / 2, b["cy"] - b["sy"] / 2), b["sx"], b["sy"],
                               facecolor=BRIGHT if wall else "#22303b", edgecolor=MUTED if not wall else BRIGHT,
                               lw=0.8, zorder=2))
    for c in course["cylinders"]:
        ax.add_patch(Circle((c["cx"], c["cy"]), c["r"], facecolor="#22303b", edgecolor=MUTED, lw=0.8, zorder=2))

    xs, ys = [r[1] for r in traj], [r[2] for r in traj]
    ax.plot(xs, ys, color=ACCENT, lw=7, alpha=0.12, solid_capstyle="round", zorder=3)  # glow
    ax.plot(xs, ys, color=ACCENT, lw=2.2, solid_capstyle="round", zorder=4)

    ax.plot(xs[0], ys[0], marker="s", ms=9, color=FG, zorder=5)
    ax.annotate("START", (xs[0], ys[0]), xytext=(8, -16), textcoords="offset points",
                color=FG, fontsize=8, family=MONO, zorder=5)
    for i, (x, y) in enumerate(course["checkpoints"], 1):
        ax.add_patch(Circle((x, y), 0.65, facecolor=BG, edgecolor=ACCENT, lw=1.6, zorder=6))
        ax.text(x, y, str(i), color=ACCENT, ha="center", va="center", fontsize=8, weight="bold",
                family=MONO, zorder=7)

    ax.set_xlim(-15.6, 15.6)
    ax.set_ylim(-15.6, 15.6)
    ax.set_aspect("equal")
    ax.set_xticks([]), ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color(BRIGHT)


# Square route figure
fig, ax = plt.subplots(figsize=(8, 8), facecolor=BG)
draw_course(ax)
ax.set_title("driven route  //  slalom -> pillar field -> gate + chicane", color=MUTED, fontsize=10,
             family=MONO, loc="left", pad=10)
fig.tight_layout()
fig.savefig(out / "nav-route.png", dpi=140, facecolor=BG)
plt.close(fig)

# 16:9 thumbnail: course on the left, stats on the right
fig = plt.figure(figsize=(16, 9), facecolor=BG)
ax = fig.add_axes([0.04, 0.06, 0.88 * 9 / 16, 0.88])
draw_course(ax)
tx = 0.04 + 0.88 * 9 / 16 + 0.05
fig.text(tx, 0.80, "ROS 2 // C++", color=ACCENT, fontsize=18, family=MONO)
fig.text(tx, 0.70, "Autonomous\nNavigation", color=FG, fontsize=40, family=MONO, weight="bold",
         va="top", linespacing=1.1)
rows = [("planner", "A* + replan"), ("control", "pure pursuit"), ("checkpoints", "5 / 5"),
        ("distance", sys.argv[3] if len(sys.argv) > 3 else f"{dist:.0f} m"),
        ("time", sys.argv[4] if len(sys.argv) > 4 else f"{traj[-1][0]:.0f} s")]
for i, (k, v) in enumerate(rows):
    y = 0.42 - i * 0.065
    fig.text(tx, y, k, color=MUTED, fontsize=17, family=MONO)
    fig.text(tx + 0.20, y, v, color=FG, fontsize=17, family=MONO)
fig.savefig(out / "nav-thumbnail.png", dpi=100, facecolor=BG)
plt.close(fig)
print("wrote", out / "nav-route.png", "and", out / "nav-thumbnail.png", f"({dist:.1f} m)")
