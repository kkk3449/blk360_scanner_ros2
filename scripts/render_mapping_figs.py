#!/usr/bin/env python3
"""Render the Chapter-6 mapping figures for the dissertation.

  fig6_mapscan.png  — scan placement: distance-only (19 scans) vs coverage-aware
  fig6_mapqual.png  — Cartographer SLAM occupancy map vs ground truth (cov_R4)

Uses the ablation run JSONs + saved maps under ~/blk360_ablation.
"""
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon as MplPolygon
import colorsys

ABL = os.path.expanduser("~/blk360_ablation")
GT = os.path.expanduser(
    "~/blk360_ros2_ws/src/blk360_bringup/maps/testroom_gt.pgm")
OUT = os.path.expanduser("~/blk360_ros2_ws/outputs_thesis")
os.makedirs(OUT, exist_ok=True)


def read_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5"
        dims = f.readline()
        while dims.startswith(b"#"):
            dims = f.readline()
        w, h = map(int, dims.split())
        f.readline()
        data = np.frombuffer(f.read(), dtype=np.uint8)
    return data.reshape(h, w)


def load_run(name):
    return json.load(open(os.path.join(ABL, f"run_{name}.json")))


def panel_scans(ax, rec, title, draw_disk=True):
    pos = rec["scan_positions"]
    R = float(rec["scan_coverage_radius_m"])
    for i, (x, y) in enumerate(pos):
        r, g, b = colorsys.hsv_to_rgb((i * 0.6180339887) % 1.0, 0.80, 0.95)
        if draw_disk and R > 0:
            ax.add_patch(Circle((x, y), R, color=(r, g, b), alpha=0.16))
        ax.plot(x, y, "o", color=(r, g, b), ms=6, mec="k", mew=0.4)
    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("x [m]")
    ax.grid(True, alpha=0.3)


def fig_scan_placement():
    runs = [
        ("dist_only_3m", "(a) Distance-only trigger\n19 scans, 0 skipped, 596 s",
         False),
        ("cov_R4", "(b) Coverage-aware, R=4 m\n3 scans, 23 skipped, 259 s", True),
        ("cov_R6", "(c) Coverage-aware, R=6 m\n3 scans, 25 skipped, 100% room", True),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    for ax, (name, title, disk) in zip(axes, runs):
        panel_scans(ax, load_run(name), title, draw_disk=disk)
    axes[0].set_ylabel("y [m]")
    plt.tight_layout()
    out = os.path.join(OUT, "fig6_mapscan.png")
    plt.savefig(out, dpi=130, bbox_inches="tight")
    plt.close()
    print("wrote", out)


def fig_map_quality():
    slam = read_pgm(os.path.expanduser("~/blk360_4scan/map_cov_R4.pgm"))
    gt = read_pgm(GT)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, img, title in [
        (axes[0], slam, "(a) Cartographer SLAM map (prior-scan radius R=4 m)\n"
         "IoU$_\\mathrm{free}$=0.965,  wall RMSE=0.31 m"),
        (axes[1], gt, "(b) Ground-truth map (room free area 85.5 m$^2$)"),
    ]:
        ax.imshow(img, cmap="gray", vmin=0, vmax=255)
        ax.set_title(title, fontsize=11)
        ax.axis("off")
    plt.tight_layout()
    out = os.path.join(OUT, "fig6_mapqual.png")
    plt.savefig(out, dpi=130, bbox_inches="tight")
    plt.close()
    print("wrote", out)


def _read_yaml_origin(yaml_path):
    res, ox, oy = 0.05, 0.0, 0.0
    for line in open(yaml_path):
        s = line.strip()
        if s.startswith("resolution:"):
            res = float(s.split(":")[1])
        elif s.startswith("origin:"):
            nums = s.split("[")[1].split("]")[0].split(",")
            ox, oy = float(nums[0]), float(nums[1])
    return res, ox, oy


def _draw_overlay(ax, abldir, name, title, draw_disk=True, traj_csv=None):
    """Draw one panel: SLAM map + scan coverage disks + robot trajectory."""
    rec = json.load(open(os.path.join(abldir, f"run_{name}.json")))
    pos = rec["scan_positions"]
    R = float(rec["scan_coverage_radius_m"])
    pgm = os.path.join(abldir, f"map_{name}.pgm")
    yaml = os.path.join(abldir, f"map_{name}.yaml")
    img = read_pgm(pgm)
    res, ox, oy = _read_yaml_origin(yaml)
    H, W = img.shape
    extent = [ox, ox + W * res, oy, oy + H * res]
    ax.imshow(img, cmap="gray", vmin=0, vmax=255, extent=extent, origin="upper")
    px = [p[0] for p in pos]
    py = [p[1] for p in pos]
    # robot trajectory (solid) if logged, else scan visit-order (dashed)
    if traj_csv and os.path.exists(traj_csv):
        import csv as _csv
        tx, ty = [], []
        for row in _csv.reader(open(traj_csv)):
            try:
                tx.append(float(row[0]))
                ty.append(float(row[1]))
            except (ValueError, IndexError):
                pass
        if tx:
            ax.plot(tx, ty, "-", color="tab:blue", lw=1.3, alpha=0.75, zorder=2,
                    label="robot path")
    else:
        ax.plot(px, py, "--", color="0.35", lw=1.1, zorder=2)
    polys = rec.get("scan_visibility_polygons")
    for i, (x, y) in enumerate(pos):
        r, g, b = colorsys.hsv_to_rgb((i * 0.6180339887 + 0.05) % 1.0, 0.8, 0.95)
        poly = polys[i] if polys and i < len(polys) else None
        if poly is not None and len(poly) >= 3:
            # occlusion-aware run: draw the ray-cast visibility polygon B(s,R)
            ax.add_patch(MplPolygon(poly, closed=True, facecolor=(r, g, b),
                                    alpha=0.22, edgecolor=(r, g, b), lw=1.2,
                                    zorder=1))
        elif draw_disk and R > 0:
            ax.add_patch(Circle((x, y), R, color=(r, g, b), alpha=0.20, zorder=1))
        ax.plot(x, y, "o", color=(r, g, b), ms=8, mec="k", mew=0.7, zorder=3)
        # BLK360 scan order label next to each scan position
        ax.annotate(f"#{i + 1}", (x, y), xytext=(7, 7),
                    textcoords="offset points", fontsize=10, fontweight="bold",
                    color="k", zorder=4,
                    bbox=dict(boxstyle="round,pad=0.15", fc="white",
                              ec=(r, g, b), lw=1.2, alpha=0.85))
    ax.set_aspect("equal")
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("x [m]")
    ax.grid(True, alpha=0.25)
    return len(pos)


def fig_overlay(name="cov_R4", abldir=ABL):
    fig, ax = plt.subplots(figsize=(10, 6))
    n = _draw_overlay(ax, abldir, name,
                      f"Coverage-aware stop-scan (R=4 m): {{}} scans")
    ax.set_title(ax.get_title().format(n))
    ax.set_ylabel("y [m]")
    fig.tight_layout()
    out = os.path.join(OUT, f"fig6_overlay_{name}.png")
    fig.savefig(out, dpi=135, bbox_inches="tight")
    plt.close(fig)
    print("wrote", out)


def fig_strategy_compare(panels):
    """panels: list of (abldir, name, title, draw_disk)."""
    fig, axes = plt.subplots(1, len(panels), figsize=(6.5 * len(panels), 5.6))
    if len(panels) == 1:
        axes = [axes]
    for ax, p in zip(axes, panels):
        abldir, name, title, disk = p[:4]
        traj = p[4] if len(p) > 4 else None
        n = _draw_overlay(ax, abldir, name, title, draw_disk=disk, traj_csv=traj)
        ax.set_title(title.format(n))
    axes[0].set_ylabel("y [m]")
    fig.tight_layout()
    out = os.path.join(OUT, "fig6_strategy.png")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print("wrote", out)


if __name__ == "__main__":
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    if mode == "overlay":
        for nm in (sys.argv[2:] or ["cov_R4"]):
            fig_overlay(nm)
    elif mode == "vis":
        VIS = os.path.expanduser("~/blk360_visrun")
        fig, ax = plt.subplots(figsize=(10, 6))
        n = _draw_overlay(ax, VIS, "vis_R10",
                          "Occlusion-aware stop-scan (visibility B(s,R), R=10 m, "
                          "tau=0.30): {} scans",
                          traj_csv=os.path.join(VIS, "traj.csv"))
        ax.set_title(ax.get_title().format(n))
        ax.set_ylabel("y [m]")
        fig.tight_layout()
        out = os.path.join(OUT, "fig_vis_overlay.png")
        fig.savefig(out, dpi=135, bbox_inches="tight")
        plt.close(fig)
        print("wrote", out)
    elif mode == "strategy":
        BLK4 = os.path.expanduser("~/blk360_4scan")
        fig_strategy_compare([
            (BLK4, "dist_only_3m",
             "(a) Distance-traveled trigger\n{} scans, no skipping", False,
             os.path.join(BLK4, "traj_dist.csv")),
            (BLK4, "cov_R4",
             "(b) Selected: prior-scan radius R=4 m\n{} scans, 91.0% covered", True,
             os.path.join(BLK4, "traj.csv")),
        ])
    elif mode == "trajfig":
        BLK4 = os.path.expanduser("~/blk360_4scan")
        fig, ax = plt.subplots(figsize=(10, 6))
        n = _draw_overlay(ax, BLK4, "cov_R4", "",
                          traj_csv=os.path.join(BLK4, "traj.csv"))
        ax.set_title("Selected stop-scan (prior-scan radius R=4 m): "
                     f"{n} BLK360 scans with the robot trajectory", fontsize=11)
        ax.set_ylabel("y [m]")
        fig.tight_layout()
        out = os.path.join(OUT, "fig6_mapscan_traj.png")
        fig.savefig(out, dpi=135, bbox_inches="tight")
        plt.close(fig)
        print("wrote", out)
    else:
        fig_scan_placement()
        fig_map_quality()
