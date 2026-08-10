#!/usr/bin/env python3
"""fig9_redbot04_platform (revision): annotated photo + dimensioned top-view.

Reviewer 3 asked for an annotated view with dimensions/scalar elements.
Dimensions from the RedBOT04 hardware spec (CNRLAB_260630_Redbot.pdf):
deck 71 x 49 cm at 31 cm height; wheelbase 28 cm; track 44 cm; wheels
D21 x 6 cm; 2D LiDAR 7 cm behind the front deck edge.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle, FancyArrowPatch
from PIL import Image

HOME = os.path.expanduser("~")
OUT = os.path.join(HOME, "blk360_ros2_ws", "outputs_thesis", "journal")
PHOTO = os.path.join(HOME, "Downloads", "redbot_04.jpg")

FS = 11.5


def callout(ax, xy, txt, txy, ha="left"):
    ax.annotate(txt, xy=xy, xytext=txy, fontsize=FS, ha=ha, va="center",
                fontweight="bold", color="#111111",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#444444",
                          lw=1.0, alpha=0.92),
                arrowprops=dict(arrowstyle="-|>", color="#222222", lw=1.6,
                                shrinkA=2, shrinkB=2))


def dim_arrow(ax, p0, p1, txt, toff=(0, 0), fs=11.5, color="#1f4e8c"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="<|-|>",
                                 mutation_scale=14, lw=1.4, color=color))
    mx, my = (p0[0] + p1[0]) / 2 + toff[0], (p0[1] + p1[1]) / 2 + toff[1]
    ax.text(mx, my, txt, fontsize=fs, ha="center", va="center", color=color,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none",
                      alpha=0.85))


def main():
    fig, (a, b) = plt.subplots(1, 2, figsize=(13.2, 5.9),
                               gridspec_kw={"width_ratios": [1.02, 1.15]})

    # ---------------- (a) annotated photo ----------------
    im = Image.open(PHOTO)
    a.imshow(im)
    a.axis("off")
    a.set_title("(a) RedBOT04 with the deck-mounted stop-and-scan sensor",
                fontsize=13)
    callout(a, (680, 330), "Leica BLK360 G1\nsurvey-grade TLS", (940, 190))
    callout(a, (640, 545), "rigid scanner mount", (880, 700))
    callout(a, (408, 268), "YDLiDAR TG30 2D LiDAR\n(Cartographer SLAM)",
            (60, 120))
    callout(a, (415, 205), "RGB-D camera", (60, 330))
    callout(a, (330, 640), "deck 71 × 49 cm\ntop at 31 cm", (55, 760))
    callout(a, (650, 900), "chassis: battery, controllers,\n4× BLDC skid-steer drive",
            (990, 1010), ha="center")

    # ---------------- (b) dimensioned top view ----------------
    b.set_xlim(-14, 87)
    b.set_ylim(-22, 63)
    b.set_aspect("equal")
    b.axis("off")
    b.set_title("(b) Top-view footprint schematic (dimensions in cm)",
                fontsize=13)
    # wheels (under the deck): wheelbase 28, track 44, wheel 21 x 6
    cx, cy = 35.5, 24.5
    for dx in (-14, 14):
        for dy in (-22, 22):
            b.add_patch(Rectangle((cx + dx - 10.5, cy + dy - 3), 21, 6,
                                  fc="#c9c9c9", ec="#555555", lw=1.2,
                                  ls="--", zorder=1))
    # deck outline 71 x 49 (front = +x)
    b.add_patch(FancyBboxPatch((0, 0), 71, 49,
                               boxstyle="round,pad=0.02,rounding_size=7",
                               fc="#e8574d", ec="#7a1f18", lw=2.0,
                               alpha=0.85, zorder=2))
    # 2D LiDAR near the front edge, 7 cm from it
    b.add_patch(Circle((64, 24.5), 3.2, fc="#241a1a", ec="k", zorder=3))
    b.text(64, 31.0, "2D LiDAR", fontsize=10.5, ha="center", zorder=4)
    b.plot([64], [24.5], "o", color="white", ms=3.2, zorder=6)
    b.add_patch(FancyArrowPatch((64, 24.5), (71, 24.5), arrowstyle="<|-|>",
                                mutation_scale=13, lw=1.6, color="#1f4e8c",
                                zorder=5, shrinkA=0, shrinkB=0))
    b.text(67.5, 20.8, "front", fontsize=10.5, ha="center", style="italic",
           color="#111111", zorder=5)
    b.text(67.5, 17.2, "7", fontsize=10.5, ha="center", fontweight="bold",
           color="#111111", zorder=5)
    # BLK360 near deck center
    b.add_patch(Circle((33, 24.5), 5.2, fc="#333333", ec="k", zorder=3))
    b.text(33, 33.5, "BLK360", fontsize=10.5, ha="center", zorder=4)
    # dimensions
    dim_arrow(b, (0, -6), (71, -6), "71", toff=(0, -3.6))
    dim_arrow(b, (77, 0), (77, 49), "49", toff=(5.2, 0))
    dim_arrow(b, (cx - 14, cy - 22), (cx + 14, cy - 22), "28 (wheelbase)",
              toff=(0, -4.2), fs=10, color="#444444")
    dim_arrow(b, (cx - 14, cy - 22), (cx - 14, cy + 22), "44\n(track)",
              toff=(-7.5, 0), fs=10, color="#444444")

    b.text(35.5, -17.5,
           "deck height 31 cm; wheels Ø21 × 6 cm;\n"
           "scanner optical center ≈ 0.5 m above the floor",
           fontsize=10.5, ha="center", color="#333333")

    fig.tight_layout(pad=0.6)
    p = f"{OUT}/fig9_redbot04_platform_annot.png"
    fig.savefig(p, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


if __name__ == "__main__":
    main()
