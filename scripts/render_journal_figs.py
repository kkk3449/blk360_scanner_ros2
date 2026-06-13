#!/usr/bin/env python3
"""Final journal figure set for the occlusion-aware visibility stop-scan paper.

Renders a numbered, consistently-styled set into outputs_thesis/journal/ from
the recorded run JSONs, saved maps and trajectory CSVs. One self-contained
script so every figure is reproducible.

  fig1_concept            disk vs visibility coverage of a single scan pose
  fig2_singleroom_sota    visibility stop-scan placement (R=6) on the testroom
  fig3_R_sweep            R = 5 / 6 / 10 m placement + coverage
  fig4_multiroom_compare  live disk vs visibility on the 3-room world
  fig5_skip_ablation      controlled paired skip-rule ablation (single + multi)
  fig6_completion         coverage-completion phase (frontier + completion scans)
  fig7_summary            grouped bar chart of the statistics

Run with the workspace python (numpy + matplotlib).
"""
import csv
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon as MplPoly
import colorsys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "blk360_stop_scan"))
from blk360_stop_scan.visibility import (  # noqa: E402
    new_visible_ratio, union_visible_mask, visible_mask)

HOME = os.path.expanduser("~")
OUT = os.path.join(HOME, "blk360_ros2_ws", "outputs_thesis", "journal")
os.makedirs(OUT, exist_ok=True)

plt.rcParams.update({
    "font.size": 11, "axes.titlesize": 12, "axes.labelsize": 11,
    "figure.dpi": 150, "savefig.dpi": 150, "font.family": "DejaVu Sans",
})


def load_map(pgm, yaml):
    with open(pgm, "rb") as f:
        assert f.readline().strip() == b"P5"
        d = f.readline()
        while d.startswith(b"#"):
            d = f.readline()
        w, h = map(int, d.split())
        f.readline()
        img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    g = np.flipud(img).astype(np.int16)
    grid = np.where(g < 100, 100, np.where(g > 250, 0, -1)).astype(np.int16)
    res, ox, oy = 0.05, 0.0, 0.0
    for line in open(yaml):
        s = line.strip()
        if s.startswith("resolution:"):
            res = float(s.split(":")[1])
        if s.startswith("origin:"):
            n = s.split("[")[1].split("]")[0].split(",")
            ox, oy = float(n[0]), float(n[1])
    return img, grid, res, (ox, oy)


def hue(i):
    return colorsys.hsv_to_rgb((i * 0.6180339887 + 0.05) % 1.0, 0.8, 0.95)


def traj_xy(path):
    tx, ty = [], []
    if not os.path.exists(path):
        return tx, ty
    for row in csv.reader(open(path)):
        try:
            tx.append(float(row[0]))
            ty.append(float(row[1]))
        except (ValueError, IndexError):
            pass
    return tx, ty


def draw_base(ax, img, res, org):
    H, W = img.shape
    ext = [org[0], org[0] + W * res, org[1], org[1] + H * res]
    ax.imshow(np.flipud(img), cmap="gray", vmin=0, vmax=255, extent=ext,
              origin="lower")
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.grid(True, alpha=0.25)
    return ext


def draw_scans(ax, grid, res, org, positions, R, model, labels=True):
    for i, (x, y) in enumerate(positions):
        r, g, b = hue(i)
        if model == "disk":
            ax.add_patch(Circle((x, y), R, color=(r, g, b), alpha=0.18, zorder=1))
        else:
            _, ep = visible_mask(grid, res, org, x, y, R)
            ax.add_patch(MplPoly(ep[::4], closed=True, facecolor=(r, g, b),
                                 alpha=0.22, edgecolor=(r, g, b), lw=1.1, zorder=1))
        ax.plot(x, y, "o", color=(r, g, b), ms=8, mec="k", mew=0.7, zorder=3)
        if labels:
            ax.annotate(f"#{i + 1}", (x, y), xytext=(6, 6),
                        textcoords="offset points", fontsize=9,
                        fontweight="bold", zorder=4,
                        bbox=dict(boxstyle="round,pad=0.12", fc="white",
                                  ec=(r, g, b), lw=1))


def los_of(grid, res, org, positions, R):
    cov = union_visible_mask(grid, res, org, positions, R)
    free = (grid >= 0) & (grid <= 25)
    return 100.0 * (cov & free).sum() / max(int(free.sum()), 1)


def candidate_path(traj_csv, interval=2.0):
    """Sample candidate poses every `interval` m of path length (shared by the
    controlled ablation and the parameter sweep)."""
    tx, ty = traj_xy(traj_csv)
    if not tx:
        return []
    cands = [(tx[0], ty[0])]
    acc = 0.0
    for i in range(1, len(tx)):
        acc += ((tx[i] - tx[i - 1]) ** 2 + (ty[i] - ty[i - 1]) ** 2) ** 0.5
        if acc >= interval:
            cands.append((tx[i], ty[i]))
            acc = 0.0
    return cands


def load_env(mapdir, names):
    """Reference grid (most-complete map) + candidate paths for each named run.
    Returns (grid, res, org, free_mask, [candidate_path, ...])."""
    best, maps = None, {}
    for nm in names:
        try:
            _, grid, res, org = load_map(f"{mapdir}/map_{nm}.pgm",
                                         f"{mapdir}/map_{nm}.yaml")
        except FileNotFoundError:
            continue
        maps[nm] = (grid, res, org)
        fa = ((grid >= 0) & (grid <= 25)).sum()
        if best is None or fa > best[0]:
            best = (fa, nm)
    grid, res, org = maps[best[1]]
    free = (grid >= 0) & (grid <= 25)
    paths = [candidate_path(f"{mapdir}/traj_{nm}.csv") for nm in names
             if os.path.exists(f"{mapdir}/traj_{nm}.csv")]
    return grid, res, org, free, [p for p in paths if len(p) >= 3]


def replay_visibility(grid, res, org, free, cands, R=6.0, tau=0.30, amin=5.0):
    """Replay the visibility skip rule on one candidate path -> (scans, LOS%)."""
    sc = []
    for xy in cands:
        if not sc:
            sc.append(xy)
            continue
        g, _, na = new_visible_ratio(grid, res, org, xy, sc, R)
        if g is None:
            continue
        if g >= tau or na >= amin:
            sc.append(xy)
    cov = union_visible_mask(grid, res, org, sc, R)
    return len(sc), 100.0 * (cov & free).sum() / max(int(free.sum()), 1)


# --------------------------------------------------------------------- fig 1
def fig1_concept():
    img, grid, res, org = load_map(
        f"{HOME}/blk360_multiroom/map_mr_vis.pgm",
        f"{HOME}/blk360_multiroom/map_mr_vis.yaml")
    s = (0.0, 0.0)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6))
    for ax, model, ttl in [
        (axes[0], "disk", "(a) Isotropic disk $B_\\mathrm{disk}(s,R)$\n"
         "claims area behind walls"),
        (axes[1], "vis", "(b) Visibility region $B(s,R)$ (proposed)\n"
         "ray-cast, line-of-sight only")]:
        draw_base(ax, img, res, org)
        draw_scans(ax, grid, res, org, [s], 6.0, model, labels=False)
        ax.set_title(ttl)
    axes[0].set_ylabel("y [m]")
    fig.suptitle("Coverage model for one BLK360 scan pose (R = 6 m)", y=1.0)
    fig.tight_layout()
    p = f"{OUT}/fig1_concept.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


# --------------------------------------------------------------------- fig 2
def fig2_singleroom_sota():
    d = json.load(open(f"{HOME}/blk360_visrun/run_vis_R6.json"))
    img, grid, res, org = load_map(f"{HOME}/blk360_visrun/map_vis_R6.pgm",
                                   f"{HOME}/blk360_visrun/map_vis_R6.yaml")
    fig, ax = plt.subplots(figsize=(9, 5.6))
    draw_base(ax, img, res, org)
    tx, ty = traj_xy(f"{HOME}/blk360_visrun/traj_R6.csv")
    ax.plot(tx, ty, "-", color="tab:blue", lw=1.1, alpha=0.7, zorder=2,
            label="robot path")
    draw_scans(ax, grid, res, org, d["scan_positions"], 6.0, "vis")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Proposed visibility stop-scan (R = 6 m): "
                 f"{d['scan_count']} scans, "
                 f"{d.get('visible_room_coverage_pct')}% LOS coverage")
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    p = f"{OUT}/fig2_singleroom_sota.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


# --------------------------------------------------------------------- fig 3
def fig3_R_sweep():
    cfgs = [("vis_R5", 5, "traj_R5.csv"), ("vis_R6", 6, "traj_R6.csv"),
            ("vis_R10", 10, "traj.csv")]
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.0))
    for ax, (name, R, traj) in zip(axes, cfgs):
        d = json.load(open(f"{HOME}/blk360_visrun/run_{name}.json"))
        img, grid, res, org = load_map(
            f"{HOME}/blk360_visrun/map_{name}.pgm",
            f"{HOME}/blk360_visrun/map_{name}.yaml")
        draw_base(ax, img, res, org)
        tx, ty = traj_xy(f"{HOME}/blk360_visrun/{traj}")
        ax.plot(tx, ty, "-", color="tab:blue", lw=1.0, alpha=0.65, zorder=2)
        draw_scans(ax, grid, res, org, d["scan_positions"], float(R), "vis")
        ax.set_title(f"R = {R} m: {d['scan_count']} scans, "
                     f"{d.get('visible_room_coverage_pct')}% LOS")
    axes[0].set_ylabel("y [m]")
    fig.suptitle("Effect of the scan range R on placement and coverage", y=1.02)
    fig.tight_layout()
    p = f"{OUT}/fig3_R_sweep.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


# --------------------------------------------------------------------- fig 4
def fig4_multiroom_compare():
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.6))
    for ax, name, model, ttl in [
        (axes[0], "mr_disk", "disk",
         "(a) Isotropic disk: {n} scans, {los:.1f}% LOS\n"
         "claims rooms behind partitions"),
        (axes[1], "mr_vis", "vis",
         "(b) Proposed visibility: {n} scans, {los:.1f}% LOS\n"
         "one scan per room")]:
        d = json.load(open(f"{HOME}/blk360_multiroom/run_{name}.json"))
        img, grid, res, org = load_map(
            f"{HOME}/blk360_multiroom/map_{name}.pgm",
            f"{HOME}/blk360_multiroom/map_{name}.yaml")
        draw_base(ax, img, res, org)
        tx, ty = traj_xy(f"{HOME}/blk360_multiroom/traj_{name}.csv")
        ax.plot(tx, ty, "-", color="tab:blue", lw=1.0, alpha=0.6, zorder=2)
        pos = d["scan_positions"]
        draw_scans(ax, grid, res, org, pos, 6.0, model)
        ax.set_title(ttl.format(n=len(pos), los=los_of(grid, res, org, pos, 6.0)))
    axes[0].set_ylabel("y [m]")
    fig.suptitle("Multi-room world (two partitions, offset doorways): "
                 "live disk vs visibility", y=1.01)
    fig.tight_layout()
    p = f"{OUT}/fig4_multiroom_compare.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


# --------------------------------------------------------------------- fig 5
def _ablation_panel(ax, mapdir, ref, traj, ttl_prefix):
    img, grid, res, org = load_map(f"{mapdir}/map_{ref}.pgm",
                                   f"{mapdir}/map_{ref}.yaml")
    free = (grid >= 0) & (grid <= 25)
    pts_x, pts_y = traj_xy(f"{mapdir}/traj_{ref}.csv")
    cands = candidate_path(f"{mapdir}/traj_{ref}.csv")

    def replay(model, R=6.0, tau=0.30, amin=5.0):
        sc = []
        for xy in cands:
            if not sc:
                sc.append(xy)
                continue
            if model == "disk":
                if min(((xy[0] - s[0]) ** 2 + (xy[1] - s[1]) ** 2) ** 0.5
                       for s in sc) >= R:
                    sc.append(xy)
            else:
                g, _, na = new_visible_ratio(grid, res, org, xy, sc, R)
                if g is None:
                    continue
                if g >= tau or na >= amin:
                    sc.append(xy)
        return sc, 100.0 * (union_visible_mask(grid, res, org, sc, R)
                            & free).sum() / max(int(free.sum()), 1)
    out = {}
    for col, model in zip(ax, ["disk", "vis"]):
        sc, los = replay(model)
        draw_base(col, img, res, org)
        col.plot(pts_x, pts_y, "-", color="tab:blue", lw=0.9, alpha=0.55, zorder=2)
        draw_scans(col, grid, res, org, sc, 6.0, model)
        tag = "disk" if model == "disk" else "visibility"
        col.set_title(f"{ttl_prefix} {tag}: {len(sc)} scans, {los:.1f}% LOS",
                      fontsize=11)
        out[model] = (len(sc), los)
    return out


def fig5_skip_ablation():
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    _ablation_panel(axes[0], f"{HOME}/blk360_sotastats", "sota_run4",
                    "traj", "(a) Single-room,")
    _ablation_panel(axes[1], f"{HOME}/blk360_mrstats", "mr_visibility_run5",
                    "traj", "(b) Multi-room,")
    for ax in (axes[0][0], axes[1][0]):
        ax.set_ylabel("y [m]")
    fig.suptitle("Controlled skip-rule ablation: identical map + identical "
                 "candidate path, only the skip decision differs", y=1.0)
    fig.tight_layout()
    p = f"{OUT}/fig5_skip_ablation.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


# --------------------------------------------------------------------- fig 6
def fig6_completion():
    d = json.load(open(f"{HOME}/blk360_ccstats/run_cc_run2.json"))
    img, grid, res, org = load_map(f"{HOME}/blk360_ccstats/map_cc_run2.pgm",
                                   f"{HOME}/blk360_ccstats/map_cc_run2.yaml")
    fig, ax = plt.subplots(figsize=(9, 5.6))
    draw_base(ax, img, res, org)
    tx, ty = traj_xy(f"{HOME}/blk360_ccstats/traj_cc_run2.csv")
    ax.plot(tx, ty, "-", color="tab:blue", lw=1.0, alpha=0.65, zorder=2,
            label="robot path")
    pos = d["scan_positions"]
    k = d.get("covering_scans", 0)
    nf = len(pos) - k
    draw_scans(ax, grid, res, org, pos, 6.0, "vis")
    for (x, y) in pos[nf:]:
        ax.plot(x, y, "*", ms=20, color="gold", mec="k", mew=1.1, zorder=5)
    ax.plot([], [], "*", ms=14, color="gold", mec="k", label="completion scan")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Coverage completion: {nf} frontier + {k} completion scans, "
                 f"{d.get('visible_room_coverage_pct')}% LOS coverage")
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    p = f"{OUT}/fig6_completion.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


# --------------------------------------------------------------------- fig 7
def fig7_summary():
    # (label, scans_mean, scans_sd, los_mean, los_sd)
    groups = [
        ("Disk\n(single-room)", 2.0, 0.0, 77.0, 4.9),
        ("Visibility\n(single-room)", 3.6, 0.9, 85.4, 6.2),
        ("Disk\n(multi-room)", 2.3, 0.5, 77.0, 6.4),
        ("Visibility\n(multi-room)", 3.6, 0.7, 84.0, 9.2),
    ]
    labels = [g[0] for g in groups]
    x = np.arange(len(groups))
    colors = ["#bbbbbb", "#2c7fb8", "#bbbbbb", "#2c7fb8"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.8))
    a1.bar(x, [g[3] for g in groups], yerr=[g[4] for g in groups],
           color=colors, capsize=5, edgecolor="k", lw=0.6)
    a1.set_xticks(x)
    a1.set_xticklabels(labels, fontsize=9)
    a1.set_ylabel("LOS coverage [%]")
    a1.set_title("(a) Line-of-sight coverage (controlled paired ablation)")
    a1.set_ylim(0, 100)
    a1.grid(True, axis="y", alpha=0.3)
    for xi, g in zip(x, groups):
        a1.text(xi, g[3] + g[4] + 1.5, f"{g[3]:.0f}", ha="center", fontsize=9)
    a2.bar(x, [g[1] for g in groups], yerr=[g[2] for g in groups],
           color=colors, capsize=5, edgecolor="k", lw=0.6)
    a2.set_xticks(x)
    a2.set_xticklabels(labels, fontsize=9)
    a2.set_ylabel("BLK360 scans")
    a2.set_title("(b) Number of stationary scans")
    a2.grid(True, axis="y", alpha=0.3)
    for xi, g in zip(x, groups):
        a2.text(xi, g[1] + g[2] + 0.06, f"{g[1]:.1f}", ha="center", fontsize=9)
    fig.suptitle("Disk vs visibility skip rule on identical paths "
                 "(N = 5 trajectories each)", y=1.02)
    fig.tight_layout()
    p = f"{OUT}/fig7_summary.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


# --------------------------------------------------------------------- fig 8
def fig8_param_sweep():
    """Deterministic sweep of the skip thresholds on fixed candidate paths.
    Shows A_min as the coverage-vs-scans knob and tau as a robust (flat) one."""
    envs = {
        "single": load_env(f"{HOME}/blk360_sotastats",
                           [f"sota_run{i}" for i in range(1, 6)]),
        "multi": load_env(f"{HOME}/blk360_mrstats",
                          [f"mr_{m}_run{i}" for m in ["disk", "visibility"]
                           for i in range(1, 6)]),
    }

    def mean_over(envname, tau, amin):
        grid, res, org, free, paths = envs[envname]
        sc, los = [], []
        for c in paths:
            n, l = replay_visibility(grid, res, org, free, c, 6.0, tau, amin)
            sc.append(n)
            los.append(l)
        return float(np.mean(sc)), float(np.mean(los))

    taus = [0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50]
    amins = [1, 2, 3, 4, 5, 6, 7, 10, 15]
    col = {"single": "#2c7fb8", "multi": "#d95f0e"}
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 5.2))

    ax1b = ax1.twinx()
    for name in ["single", "multi"]:
        s = [mean_over(name, t, 5.0) for t in taus]
        ax1.plot(taus, [x[1] for x in s], "-o", color=col[name],
                 label=f"{name} — LOS")
        ax1b.plot(taus, [x[0] for x in s], "--s", color=col[name],
                  alpha=0.45, ms=4)
    ax1.axvline(0.30, ls=":", color="0.4", lw=1.2)
    ax1.text(0.31, 78.6, "τ = 0.30\n(operating point)", fontsize=8.5, color="0.3")
    ax1.set_xlabel("τ  (min new-visible ratio)")
    ax1.set_ylabel("LOS coverage [%]  (solid ●)")
    ax1b.set_ylabel("BLK360 scans  (dashed ■)")
    ax1.set_title("(a) τ sweep   (A_min = 5 m²)")
    ax1.set_ylim(78, 92)
    ax1b.set_ylim(2, 8)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=9, loc="upper right")

    ax2b = ax2.twinx()
    for name in ["single", "multi"]:
        s = [mean_over(name, 0.30, a) for a in amins]
        ax2.plot(amins, [x[1] for x in s], "-o", color=col[name],
                 label=f"{name} — LOS")
        ax2b.plot(amins, [x[0] for x in s], "--s", color=col[name],
                  alpha=0.45, ms=4)
    ax2.axvline(5.0, ls=":", color="0.4", lw=1.2)
    ax2.text(5.3, 90.3, "A_min = 5 m²\n(knee)", fontsize=8.5, color="0.3")
    ax2.set_xlabel("A_min  (min new-visible area) [m²]")
    ax2.set_ylabel("LOS coverage [%]  (solid ●)")
    ax2b.set_ylabel("BLK360 scans  (dashed ■)")
    ax2.set_title("(b) A_min sweep   (τ = 0.30)")
    ax2.set_ylim(78, 92)
    ax2b.set_ylim(2, 8)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=9, loc="lower left")

    fig.suptitle("Parameter sweep (deterministic replay, N=5 paths/env): "
                 "A_min sets the coverage–scans trade-off, τ is robust",
                 y=1.03, fontsize=12)
    fig.tight_layout()
    p = f"{OUT}/fig8_param_sweep.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


if __name__ == "__main__":
    fig1_concept()
    fig2_singleroom_sota()
    fig3_R_sweep()
    fig4_multiroom_compare()
    fig5_skip_ablation()
    fig6_completion()
    fig7_summary()
    fig8_param_sweep()
    print("\nAll journal figures in", OUT)
