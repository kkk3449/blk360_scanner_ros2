#!/usr/bin/env python3
"""fig0_architecture: system architecture diagram (revision version).

Reviewer 1 (minor 14) asked for larger labels and a simpler diagram. Compact
two-row layout: exploration pipeline on top, stop-and-scan sequencer +
registered output on the bottom, fonts sized for \\linewidth printing.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Patch

HOME = os.path.expanduser("~")
OUT = os.path.join(HOME, "blk360_ros2_ws", "outputs_thesis", "journal")

C_EXPL = "#e8f0f8"   # exploration (existing)
C_EXPL_E = "#4a7fb5"
C_VIS = "#a9cdf2"    # visibility core (this work)
C_VIS_E = "#1f5fa8"
C_SEQ = "#f7e8c8"    # sequencer
C_SEQ_E = "#c8922a"
C_OUTB = "#e4f0e2"   # output
C_OUT_E = "#4e8a4a"
C_GRID = "#eeeae4"
C_GRID_E = "#8a8378"

FS_HEAD = 15.5       # container headers
FS_MAIN = 15.0       # box main label
FS_SUB = 12.5        # box sub label
FS_EDGE = 12.5       # arrow labels


def box(ax, x, y, w, h, main, sub, fc, ec, lw=2.0, main_c="#222222"):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0.012",
                                fc=fc, ec=ec, lw=lw, zorder=3))
    cy = y + h * (0.60 if sub else 0.5)
    ax.text(x + w / 2, cy, main, ha="center", va="center",
            fontsize=FS_MAIN, fontweight="bold", color=main_c, zorder=4)
    if sub:
        ax.text(x + w / 2, y + h * 0.26, sub, ha="center", va="center",
                fontsize=FS_SUB, style="italic", color="#666666", zorder=4)


def arrow(ax, p0, p1, color="#222222", lw=2.4, style="-", rad=0.0,
          label=None, lxy=None, lcolor=None, ha="center"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>",
                                 mutation_scale=22, lw=lw, color=color,
                                 linestyle=style, zorder=5,
                                 connectionstyle=f"arc3,rad={rad}"))
    if label:
        ax.text(*lxy, label, fontsize=FS_EDGE, style="italic",
                color=lcolor or color, ha=ha, va="center", zorder=6)


def main():
    fig, ax = plt.subplots(figsize=(14.2, 6.4))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 45)
    ax.axis("off")

    # ---- containers -----------------------------------------------------
    ax.add_patch(FancyBboxPatch((1.5, 25.5), 97, 17.5,
                                boxstyle="round,pad=0.012", fc="#f4f8fc",
                                ec=C_EXPL_E, lw=1.6, linestyle="--", zorder=1))
    ax.text(3.5, 40.9, "FRONTIER EXPLORATION LAYER (existing)",
            fontsize=FS_HEAD, fontweight="bold", color=C_EXPL_E, zorder=2)
    ax.add_patch(FancyBboxPatch((1.5, 2.0), 71, 20,
                                boxstyle="round,pad=0.012", fc="#fdf8ee",
                                ec=C_SEQ_E, lw=1.6, linestyle="--", zorder=1))
    ax.text(3.5, 19.9, "STOP-AND-SCAN SEQUENCER",
            fontsize=FS_HEAD, fontweight="bold", color=C_SEQ_E, zorder=2)

    # ---- top row: exploration pipeline ---------------------------------
    bw, bh, y0 = 21.0, 11.0, 27.5
    xs = [3.5, 27.5, 51.5, 75.5]
    box(ax, xs[0], y0, bw, bh, "Online 2D\noccupancy grid", "SLAM, onboard laser",
        C_GRID, C_GRID_E)
    box(ax, xs[1], y0, bw, bh, "Frontier\ndetection", "decision map + WFD",
        C_EXPL, C_EXPL_E)
    box(ax, xs[2], y0, bw, bh, "Goal cost\n& ordering", "gain / path / MRTSP",
        C_EXPL, C_EXPL_E)
    box(ax, xs[3], y0, bw, bh, "Navigation\ngoal dispatch", "Nav2, MPPI",
        C_EXPL, C_EXPL_E)
    for a, b in zip(xs[:-1], xs[1:]):
        arrow(ax, (a + bw + 0.4, y0 + bh / 2), (b - 0.4, y0 + bh / 2))

    # ---- bottom row: sequencer + output --------------------------------
    bw2, bh2, y1 = 20.0, 11.5, 4.5
    x_gate, x_vis, x_dec, x_out = 3.5, 27.0, 50.5, 76.0
    box(ax, x_gate, y1, bw2, bh2, "Distance gate",
        "candidate every d m", C_SEQ, C_SEQ_E)
    box(ax, x_vis, y1, bw2, bh2, "Ray-cast visibility\n$B(s,R)$",
        "occlusion-aware (this work)", C_VIS, C_VIS_E, lw=3.2,
        main_c="#123f70")
    box(ax, x_dec, y1, bw2, bh2, "Scan / skip decision",
        "$g \\geq \\tau$  or  $a \\geq A_{\\min}$", C_SEQ, C_SEQ_E)
    box(ax, x_out, y1, bw2, bh2, "Registered 3D\npoint cloud",
        "offline registration", C_OUTB, C_OUT_E)
    arrow(ax, (x_gate + bw2 + 0.4, y1 + bh2 / 2), (x_vis - 0.4, y1 + bh2 / 2))
    arrow(ax, (x_vis + bw2 + 0.4, y1 + bh2 / 2), (x_dec - 0.4, y1 + bh2 / 2))
    arrow(ax, (x_dec + bw2 + 0.4, y1 + bh2 / 2), (x_out - 0.4, y1 + bh2 / 2),
          color=C_SEQ_E, label="stationary scans", lxy=(73.3, y1 - 1.6),
          lcolor="#a8791d")

    # dispatch -> distance gate (robot pose stream)
    arrow(ax, (xs[3] + 2.5, y0 - 0.7), (x_gate + bw2 * 0.50, y1 + bh2 + 0.6),
          rad=-0.04, label="robot pose while exploring",
          lxy=(55.0, 24.8), lcolor="#333333")
    # visibility -> dispatch (preemption, dashed)
    arrow(ax, (x_vis + bw2 * 0.30, y1 + bh2 + 0.5), (xs[1] + bw * 0.55, y0 - 0.5),
          color=C_VIS_E, style="--", rad=0.18,
          label="pause / resume", lxy=(24.5, 23.2), ha="right")

    fig.tight_layout(pad=0.4)
    p = f"{OUT}/fig0_architecture.png"
    fig.savefig(p, dpi=170, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


if __name__ == "__main__":
    main()
