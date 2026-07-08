#!/usr/bin/env python3
"""Redbot-parameterized simulation of visibility stop-and-scan in the physical
test room (geometry from the real registered BLK360 cloud, built by
build_testroom_map.py).

Produces the field-experiment figure set + numbers for the paper's real-hardware
section (disk vs visibility on the test-room geometry), in the same style as the
controlled-ablation coverage figures:

  figA_testroom_field   two panels, disk vs visibility: robot path, scan poses,
                        ray-cast coverage overlay, "N scans, X% LOS".

Everything is a *simulation* on the real room geometry; the registration numbers
in the paper come from the separate physical Cyclone campaign.
"""
import os
import sys
from collections import deque

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import ndimage

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "blk360_stop_scan"))
sys.path.insert(0, os.path.dirname(__file__))
from render_journal_figs import (load_map, draw_base, draw_scans,  # noqa: E402
                                 hue)
from blk360_stop_scan.visibility import (  # noqa: E402
    new_visible_ratio, union_visible_mask)

HOME = os.path.expanduser("~")
MAPDIR = os.path.join(HOME, "blk360_fieldsim")
OUT = os.path.join(HOME, "blk360_ros2_ws", "outputs_thesis", "journal")
os.makedirs(OUT, exist_ok=True)

R, TAU, AMIN = 6.0, 0.30, 5.0
plt.rcParams.update({"font.size": 11, "axes.titlesize": 12,
                     "figure.dpi": 150, "savefig.dpi": 150})


def w2c(x, y, res, org):
    return int((x - org[0]) / res), int((y - org[1]) / res)   # col, row


def c2w(c, r, res, org):
    return org[0] + (c + 0.5) * res, org[1] + (r + 0.5) * res


def nearest_free(free, c, r):
    if free[r, c]:
        return c, r
    ys, xs = np.where(free)
    d = (xs - c) ** 2 + (ys - r) ** 2
    i = int(np.argmin(d))
    return int(xs[i]), int(ys[i])


def bfs_route(passable, start, goal):
    """4/8-connected BFS shortest path between grid cells on a passable mask.
    start/goal are (col,row). Returns list of (col,row)."""
    H, W = passable.shape
    sc, sr = start
    gc, gr = goal
    if not passable[sr, sc]:
        return [start]
    prev = {(sc, sr): None}
    q = deque([(sc, sr)])
    nb = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while q:
        c, r = q.popleft()
        if (c, r) == (gc, gr):
            break
        for dc, dr in nb:
            nc, nr = c + dc, r + dr
            if 0 <= nc < W and 0 <= nr < H and passable[nr, nc] \
                    and (nc, nr) not in prev:
                prev[(nc, nr)] = (c, r)
                q.append((nc, nr))
    if (gc, gr) not in prev:
        return [start, goal]
    path, cur = [], (gc, gr)
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    return path[::-1]


# exploration waypoints in open areas of the room (world frame)
WPS = [(-5.2, 0.3), (-4.2, 3.2), (-2.0, 2.0), (-1.0, 3.6), (1.2, 3.4),
       (1.6, 1.2), (0.5, -0.8), (2.4, -2.6), (4.6, -0.6), (3.6, 2.2),
       (6.2, 1.0), (8.6, 2.2), (9.2, -0.6), (8.6, -2.8), (6.5, -3.2),
       (4.2, -3.2), (-3.5, -2.6), (-5.5, -1.8)]


def make_trajectory(free, res, org, variant=0):
    """Route an exploration-style tour through open free space. `variant` yields
    distinct but plausible frontier sweeps (different start / order / small
    jitter) so results can be averaged over runs, as the paper does. variant=0
    is the representative run saved to traj_testroom.csv."""
    passable = ndimage.binary_erosion(free, structure=np.ones((3, 3)),
                                      iterations=5)
    if passable.sum() < 100:
        passable = free
    wps = list(WPS)
    if variant:
        rot = variant % len(wps)
        wps = wps[rot:] + wps[:rot]
        if variant % 2:
            wps = wps[::-1]
        jit = 0.35 * ((variant % 3) - 1)
        wps = [(x + jit, y - jit) for (x, y) in wps]
    cells = []
    for x, y in wps:
        c, r = w2c(x, y, res, org)
        c = min(max(c, 0), free.shape[1] - 1)
        r = min(max(r, 0), free.shape[0] - 1)
        cells.append(nearest_free(passable, c, r))
    route = [cells[0]]
    for nxt in cells[1:]:
        route.extend(bfs_route(passable, route[-1], nxt)[1:])
    rawx = np.array([c2w(c, r, res, org)[0] for c, r in route])
    rawy = np.array([c2w(c, r, res, org)[1] for c, r in route])
    # smooth for a natural drive line, but keep every sample inside the
    # collision-free corridor (revert to the BFS point if smoothing would cut a
    # corner through a wall/obstacle) -- the path never crosses an obstacle
    sx = np.convolve(rawx, np.ones(5) / 5, "same")
    sy = np.convolve(rawy, np.ones(5) / 5, "same")
    xs, ys = rawx.copy(), rawy.copy()
    H, W = passable.shape
    for i in range(len(route)):
        c = int((sx[i] - org[0]) / res)
        r = int((sy[i] - org[1]) / res)
        if 0 <= r < H and 0 <= c < W and passable[r, c]:
            xs[i], ys[i] = sx[i], sy[i]
    traj = list(zip(xs, ys))
    if variant == 0:
        with open(os.path.join(MAPDIR, "traj_testroom.csv"), "w") as f:
            for x, y in traj:
                f.write("%.3f,%.3f\n" % (x, y))
    return traj


def candidate_path(traj, interval=2.0):
    cands = [traj[0]]
    acc = 0.0
    for i in range(1, len(traj)):
        acc += ((traj[i][0] - traj[i - 1][0]) ** 2
                + (traj[i][1] - traj[i - 1][1]) ** 2) ** 0.5
        if acc >= interval:
            cands.append(traj[i])
            acc = 0.0
    return cands


def replay(model, cands, grid, res, org, free):
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
            if g >= TAU or na >= AMIN:
                sc.append(xy)
    cov = union_visible_mask(grid, res, org, sc, R)
    los = 100.0 * (cov & free).sum() / max(int(free.sum()), 1)
    return sc, los, cov


def build_slam_map(gt_occ, trajs, res, org, lidar_range=8.0, num_rays=360):
    """Simulate a 2D lidar (like the YDLiDAR / Cartographer front end) sweeping
    along the exploration trajectory over the solid ground-truth walls, and
    accumulate an occupancy map exactly as SLAM would: cells along each ray up
    to the first hit are marked free (seen), the hit cell is marked occupied
    (a *surface* return -- so obstacle interiors and occluded pockets stay
    unknown), everything untouched stays unknown. This reproduces the authentic
    RViz look: hollow obstacle boxes, ragged range-limited edges, dark unknown
    pockets. `gt_occ` is a bool ground-truth occupancy (True = wall/furniture).
    Returns a grid in nav_msgs convention (-1 unknown, 0 free, 100 occupied)."""
    H, W = gt_occ.shape
    seen_free = np.zeros((H, W), bool)
    seen_occ = np.zeros((H, W), bool)
    step = res * 0.5
    n_steps = max(int(lidar_range / step), 1)
    ang = np.linspace(0.0, 2 * np.pi, num_rays, endpoint=False)
    radii = (np.arange(n_steps) + 1.0) * step
    ca, sa = np.cos(ang)[:, None], np.sin(ang)[:, None]
    for traj in trajs:
        poses = traj[::5]                      # ~every 0.25 m along the path
        for (x, y) in poses:
            px = x + ca * radii[None, :]
            py = y + sa * radii[None, :]
            col = np.floor((px - org[0]) / res).astype(np.int32)
            row = np.floor((py - org[1]) / res).astype(np.int32)
            inb = (col >= 0) & (col < W) & (row >= 0) & (row < H)
            hit = np.zeros(px.shape, bool)
            hit[inb] = gt_occ[row[inb], col[inb]]
            blocked = ~inb | hit
            any_hit = blocked.any(1)
            first = np.where(any_hit, blocked.argmax(1), n_steps)
            idx = np.arange(n_steps)[None, :]
            free_sel = (idx < first[:, None]) & inb
            seen_free[row[free_sel], col[free_sel]] = True
            hk = np.where(any_hit)[0]
            fk = first[hk]
            valid = fk < n_steps
            rr = row[hk[valid], fk[valid]]
            cc = col[hk[valid], fk[valid]]
            good = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < W)
            seen_occ[rr[good], cc[good]] = True
    # tidy thin single-ray streaks (Cartographer's probabilistic filter does the
    # same): drop 1-cell free fingers, then keep the main connected free area
    seen_free = ndimage.binary_opening(seen_free, np.ones((3, 3)), iterations=1)
    seen_free = ndimage.binary_closing(seen_free, np.ones((3, 3)), iterations=1)
    lab, n = ndimage.label(seen_free, np.ones((3, 3)))
    if n:
        sz = ndimage.sum(np.ones_like(lab), lab, index=np.arange(1, n + 1))
        seen_free = lab == (int(np.argmax(sz)) + 1)
    # Close the room with its real walls: any ground-truth wall/furniture cell
    # bordering the explored free area is drawn as an occupied return, so the
    # navigable region is fully enclosed (no open gaps where a wall exists).
    border = ndimage.binary_dilation(seen_free, np.ones((3, 3)), iterations=1)
    seen_occ = (seen_occ | (gt_occ & border)) & ~seen_free
    grid = np.full((H, W), -1, np.int16)
    grid[seen_free] = 0
    grid[seen_occ] = 100
    return grid


def rviz_base(ax, grid, res, org):
    """Render the occupancy grid in the RViz / Nav2 lidar-map palette:
    navigable free = light grey, unknown / unscannable = dark grey,
    occupied = black."""
    H, W = grid.shape
    shade = np.full((H, W), 0.50)            # unknown -> dark grey
    shade[(grid >= 0) & (grid <= 25)] = 0.88  # free -> light grey
    shade[grid >= 65] = 0.18                  # occupied -> dark (not pure black)
    ext = [org[0], org[0] + W * res, org[1], org[1] + H * res]
    ax.imshow(shade, cmap="gray", vmin=0, vmax=1, extent=ext, origin="lower",
              interpolation="nearest")
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.grid(True, alpha=0.15)
    return ext


N_RUNS = 8


def main():
    _, gt_grid, res, org = load_map(f"{MAPDIR}/map_testroom.pgm",
                                    f"{MAPDIR}/map_testroom.yaml")
    gt_free = (gt_grid >= 0) & (gt_grid <= 25)     # ground-truth navigable area
    gt_occ = gt_grid >= 65                          # solid walls + furniture
    m = np.load(f"{MAPDIR}/masks.npz")
    wall = m["wall_ring"]                            # room perimeter walls

    # ---- exploration trajectories (paper method: N frontier runs) ----
    trajs = [make_trajectory(gt_free, res, org, variant=v)
             for v in range(N_RUNS)]

    # ---- authentic SLAM reference map: a 2D lidar sweeps every run's path over
    # the solid ground truth, accumulated as Cartographer would (hollow
    # obstacles, ragged edges, unknown pockets) ----
    grid = build_slam_map(gt_occ, trajs, res, org, lidar_range=12.0)
    # the enclosing walls are large flat surfaces the lidar sees along their
    # whole length -> draw the perimeter as a continuous scanned wall (thin,
    # 1-cell, like a real occupancy grid)
    grid[wall] = 100
    free = (grid >= 0) & (grid <= 25)
    free_m2 = free.sum() * res * res

    # ---- average disk vs visibility over the N runs on the common SLAM map ----
    runs = []
    for traj in trajs:
        cands = candidate_path(traj, 2.0)
        dsc, dlos, _ = replay("disk", cands, grid, res, org, free)
        vsc, vlos, _ = replay("vis", cands, grid, res, org, free)
        runs.append(dict(traj=traj, dsc=dsc, dlos=dlos, vsc=vsc, vlos=vlos))

    def ms(vals):
        a = np.array(vals, float)
        return a.mean(), (a.std(ddof=1) if len(a) > 1 else 0.0)

    dsc_m, dsc_s = ms([len(r["dsc"]) for r in runs])
    dlos_m, dlos_s = ms([r["dlos"] for r in runs])
    vsc_m, vsc_s = ms([len(r["vsc"]) for r in runs])
    vlos_m, vlos_s = ms([r["vlos"] for r in runs])
    print("=== Test-room field simulation, N=%d runs, %.1f m^2 free ==="
          % (N_RUNS, free_m2))
    print("disk       : %.1f+/-%.1f scans   %.1f+/-%.1f %% LOS"
          % (dsc_m, dsc_s, dlos_m, dlos_s))
    print("visibility : %.1f+/-%.1f scans   %.1f+/-%.1f %% LOS"
          % (vsc_m, vsc_s, vlos_m, vlos_s))

    # ---- representative run for the figure: closest to BOTH mean LOS values,
    # so the figure's numbers track the reported means ----
    rep = min(runs, key=lambda r: abs(r["dlos"] - dlos_m)
              + abs(r["vlos"] - vlos_m))
    tx = [p[0] for p in rep["traj"]]
    ty = [p[1] for p in rep["traj"]]
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.4))
    for ax, model, tag, (sc, los) in [
            (axes[0], "disk", "(a) Isotropic disk",
             (rep["dsc"], rep["dlos"])),
            (axes[1], "vis", "(b) Ray-cast visibility (proposed)",
             (rep["vsc"], rep["vlos"]))]:
        rviz_base(ax, grid, res, org)
        ax.plot(tx, ty, "-", color="#6a3d9a", lw=1.4, alpha=0.85, zorder=2,
                label="robot path")
        draw_scans(ax, grid, res, org, sc, R, model)
        ax.set_title("%s: %d scans, %.1f%% LOS" % (tag, len(sc), los))
        ax.legend(loc="upper right", fontsize=8)
    axes[0].set_ylabel("y [m]")
    fig.suptitle("Test-room field simulation (Redbot on real BLK360 room "
                 "geometry): disk vs. visibility stop-and-scan (R = 6 m)",
                 y=1.02)
    fig.tight_layout()
    p = f"{OUT}/figA_testroom_field.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)
    return dict(free_m2=free_m2, disk=(dsc_m, dsc_s, dlos_m, dlos_s),
                vis=(vsc_m, vsc_s, vlos_m, vlos_s))


def main_gz():
    """Render figA from a REAL Gazebo capture run (run_testroom_field.sh):
    real Cartographer SLAM map + real Nav2/frontier trajectory. Disk and
    visibility are replayed on the real map + real candidate path (controlled
    ablation), so the only thing that differs between panels is the skip rule."""
    import csv
    import json
    # allow an explicit capture dir: `--gz <dir>`; default to the newest run
    gzdir = "blk360_fieldsim_gz2"
    for a in sys.argv:
        if a.startswith("blk360_fieldsim_gz"):
            gzdir = a
    GZ = os.path.join(HOME, gzdir)
    _, grid, res, org = load_map(f"{GZ}/map_field.pgm", f"{GZ}/map_field.yaml")
    free = (grid >= 0) & (grid <= 25)
    free_m2 = free.sum() * res * res
    tx, ty = [], []
    with open(f"{GZ}/traj.csv") as f:
        for row in csv.reader(f):
            try:
                tx.append(float(row[0]))
                ty.append(float(row[1]))
            except (ValueError, IndexError):
                pass
    traj = list(zip(tx, ty))
    cands = candidate_path(traj, 2.0)
    dsc, dlos, _ = replay("disk", cands, grid, res, org, free)
    vsc, vlos, _ = replay("vis", cands, grid, res, org, free)
    # the live run itself placed scans under the visibility policy:
    live = []
    try:
        d = json.load(open(f"{GZ}/run_field.json"))
        live = [(float(x), float(y)) for x, y in d.get("scan_positions", [])]
    except (FileNotFoundError, ValueError, KeyError):
        pass
    print("=== Test-room REAL Gazebo run, %.1f m^2 free, %d traj pts ==="
          % (free_m2, len(traj)))
    print("disk (replay)       : %d scans  %.1f %% LOS" % (len(dsc), dlos))
    print("visibility (replay) : %d scans  %.1f %% LOS" % (len(vsc), vlos))
    print("visibility (live placement) : %d scans" % len(live))

    fig, axes = plt.subplots(1, 2, figsize=(15, 5.4))
    for ax, model, tag, (sc, los) in [
            (axes[0], "disk", "(a) Isotropic disk", (dsc, dlos)),
            (axes[1], "vis", "(b) Ray-cast visibility (proposed)", (vsc, vlos))]:
        rviz_base(ax, grid, res, org)
        ax.plot(tx, ty, "-", color="#6a3d9a", lw=1.4, alpha=0.85, zorder=2,
                label="robot path")
        draw_scans(ax, grid, res, org, sc, R, model)
        ax.set_title("%s: %d scans, %.1f%% LOS" % (tag, len(sc), los))
        ax.legend(loc="upper right", fontsize=8)
    axes[0].set_ylabel("y [m]")
    fig.suptitle("Test-room field experiment (real Gazebo run: frontier "
                 "exploration + Nav2 + Cartographer): disk vs. visibility "
                 "stop-and-scan (R = 6 m)", y=1.02)
    fig.tight_layout()
    p = f"{OUT}/figA_testroom_field.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)
    return dict(free_m2=free_m2, disk=(len(dsc), dlos), vis=(len(vsc), vlos),
                live=len(live))


def main_nruns():
    """Aggregate N independent real Gazebo capture runs (run_field_nruns.sh):
    replay disk & visibility on each run's own SLAM map + trajectory, report
    mean +/- sd, and render the representative run (closest to both means)."""
    import csv
    # Each run's Cartographer map frame starts at the robot spawn (yaw=0, verified
    # traj[0]~(0,0)), so world = map + spawn. We put every trajectory into the
    # world frame and replay all of them on ONE common reference map (the most
    # complete real SLAM map, run4) also placed in the world frame -- the paper's
    # "re-evaluate every run on a single common reference map" methodology.
    SP = {"run1": (1.97, 0.28), "run3": (6.48, 0.48), "run4": (-0.02, -2.52),
          "run5": (2.98, 2.98), "gz2": (2.03, 0.28)}
    DIR = {"run1": "blk360_fieldsim_gz2_run1", "run3": "blk360_fieldsim_gz2_run3",
           "run4": "blk360_fieldsim_gz2_run4", "run5": "blk360_fieldsim_gz2_run5",
           "gz2": "blk360_fieldsim_gz2"}

    def load_traj(tag):
        tx, ty = [], []
        f = os.path.join(HOME, DIR[tag], "traj.csv")
        if os.path.exists(f):
            for row in csv.reader(open(f)):
                try:
                    tx.append(float(row[0]) + SP[tag][0])
                    ty.append(float(row[1]) + SP[tag][1])
                except (ValueError, IndexError):
                    pass
        return tx, ty

    # common reference: run4's real SLAM map, shifted into the world frame
    _, grid, res, org0 = load_map(
        os.path.join(HOME, DIR["run4"], "map_field.pgm"),
        os.path.join(HOME, DIR["run4"], "map_field.yaml"))
    org = (org0[0] + SP["run4"][0], org0[1] + SP["run4"][1])
    free = (grid >= 0) & (grid <= 25)
    print("common reference = run4 real SLAM map (%.1f m^2 free), world frame"
          % (free.sum() * res * res))

    # a valid run must have actually explored the room; runs whose frontier
    # exploration terminated early (robot barely moved) are excluded as failed
    # trials, not policy samples.
    MIN_TRAJ = 150
    runs = []
    for tag in ["run1", "run3", "run4", "run5", "gz2"]:
        tx, ty = load_traj(tag)
        if len(tx) < MIN_TRAJ:
            print("skip %s (traj %d pts < %d: early-terminated run)"
                  % (tag, len(tx), MIN_TRAJ))
            continue
        cands = candidate_path(list(zip(tx, ty)), 2.0)
        dsc, dlos, _ = replay("disk", cands, grid, res, org, free)
        vsc, vlos, _ = replay("vis", cands, grid, res, org, free)
        runs.append(dict(n=tag, tx=tx, ty=ty, dsc=dsc, dlos=dlos,
                         vsc=vsc, vlos=vlos))
        print("%-5s: disk %d/%.1f%%  vis %d/%.1f%%"
              % (tag, len(dsc), dlos, len(vsc), vlos))

    def ms(vals):
        a = np.array(vals, float)
        return a.mean(), (a.std(ddof=1) if len(a) > 1 else 0.0)
    dsc_m, dsc_s = ms([len(r["dsc"]) for r in runs])
    dlos_m, dlos_s = ms([r["dlos"] for r in runs])
    vsc_m, vsc_s = ms([len(r["vsc"]) for r in runs])
    vlos_m, vlos_s = ms([r["vlos"] for r in runs])
    print("\n=== Test-room REAL Gazebo, N=%d runs ===" % len(runs))
    print("disk       : %.1f+/-%.1f scans   %.1f+/-%.1f %% LOS"
          % (dsc_m, dsc_s, dlos_m, dlos_s))
    print("visibility : %.1f+/-%.1f scans   %.1f+/-%.1f %% LOS"
          % (vsc_m, vsc_s, vlos_m, vlos_s))

    rep = min(runs, key=lambda r: abs(r["dlos"] - dlos_m)
              + abs(r["vlos"] - vlos_m))
    print("representative run:", rep["n"])
    # figure uses the common reference map (grid/res/org from run4, world frame)
    fig, axes = plt.subplots(1, 2, figsize=(15, 5.4))
    for ax, model, tag, (sc, los) in [
            (axes[0], "disk", "(a) Isotropic disk", (rep["dsc"], rep["dlos"])),
            (axes[1], "vis", "(b) Ray-cast visibility (proposed)",
             (rep["vsc"], rep["vlos"]))]:
        rviz_base(ax, grid, res, org)
        ax.plot(rep["tx"], rep["ty"], "-", color="#6a3d9a", lw=1.4, alpha=0.85,
                zorder=2, label="robot path")
        draw_scans(ax, grid, res, org, sc, R, model)
        ax.set_title("%s: %d scans, %.1f%% LOS" % (tag, len(sc), los))
        ax.legend(loc="upper right", fontsize=8)
    axes[0].set_ylabel("y [m]")
    fig.suptitle("Test-room field experiment (real Gazebo: frontier exploration "
                 "+ Nav2 + Cartographer, N=%d runs): disk vs. visibility "
                 "stop-and-scan (R = 6 m)" % len(runs), y=1.02)
    fig.tight_layout()
    p = f"{OUT}/figA_testroom_field.png"
    fig.savefig(p, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)
    return dict(n=len(runs), disk=(dsc_m, dsc_s, dlos_m, dlos_s),
                vis=(vsc_m, vsc_s, vlos_m, vlos_s))


def _cov_overlay(ax, cov, free, res, org, color=(0.15, 0.65, 0.30)):
    """Tint the LOS-covered free area (cov & free) as a translucent overlay so
    each figure reads as a *coverage* map, not just scan circles."""
    H, W = cov.shape
    rgba = np.zeros((H, W, 4))
    m = cov & free
    rgba[m] = (*color, 0.32)
    ext = [org[0], org[0] + W * res, org[1], org[1] + H * res]
    ax.imshow(rgba, extent=ext, origin="lower", interpolation="nearest",
              zorder=1.5)


def main_perrun():
    """Render, for EVERY valid run, a separate disk figure and a separate
    visibility figure (robot path + scan poses + per-scan patches + the union
    LOS-coverage overlay). One PNG per (run, model), all on the common
    reference map -- so e.g. 4 valid runs -> 4 disk + 4 visibility images."""
    import csv
    SP = {"run1": (1.97, 0.28), "run3": (6.48, 0.48), "run4": (-0.02, -2.52),
          "run5": (2.98, 2.98), "gz2": (2.03, 0.28)}
    DIR = {"run1": "blk360_fieldsim_gz2_run1", "run3": "blk360_fieldsim_gz2_run3",
           "run4": "blk360_fieldsim_gz2_run4", "run5": "blk360_fieldsim_gz2_run5",
           "gz2": "blk360_fieldsim_gz2"}

    def load_traj(tag):
        tx, ty = [], []
        f = os.path.join(HOME, DIR[tag], "traj.csv")
        if os.path.exists(f):
            for row in csv.reader(open(f)):
                try:
                    tx.append(float(row[0]) + SP[tag][0])
                    ty.append(float(row[1]) + SP[tag][1])
                except (ValueError, IndexError):
                    pass
        return tx, ty

    _, grid, res, org0 = load_map(
        os.path.join(HOME, DIR["run4"], "map_field.pgm"),
        os.path.join(HOME, DIR["run4"], "map_field.yaml"))
    org = (org0[0] + SP["run4"][0], org0[1] + SP["run4"][1])
    free = (grid >= 0) & (grid <= 25)
    free_m2 = free.sum() * res * res
    print("common reference = run4 real SLAM map (%.1f m^2 free), world frame"
          % free_m2)

    MIN_TRAJ = 150
    outdir = os.path.join(OUT, "fieldsim_perrun")
    os.makedirs(outdir, exist_ok=True)
    tags = ["run1", "run4", "run5", "gz2"]     # the N=4 valid runs
    order = {t: i + 1 for i, t in enumerate(tags)}
    written = []
    for tag in tags:
        tx, ty = load_traj(tag)
        if len(tx) < MIN_TRAJ:
            print("skip %s (traj %d < %d)" % (tag, len(tx), MIN_TRAJ))
            continue
        cands = candidate_path(list(zip(tx, ty)), 2.0)
        idx = order[tag]
        for model, mlabel, mtag in [
                ("disk", "Isotropic disk", "disk"),
                ("vis", "Ray-cast visibility (proposed)", "visibility")]:
            sc, los, cov = replay(model, cands, grid, res, org, free)
            fig, ax = plt.subplots(figsize=(8.2, 6.4))
            rviz_base(ax, grid, res, org)
            _cov_overlay(ax, cov, free, res, org)
            ax.plot(tx, ty, "-", color="#6a3d9a", lw=1.5, alpha=0.85, zorder=2,
                    label="robot path")
            draw_scans(ax, grid, res, org, sc, R, model)
            ax.set_ylabel("y [m]")
            ax.set_title("Run %d (%s) -- %s:\n%d scans, %.1f%% LOS coverage"
                         % (idx, tag, mlabel, len(sc), los))
            ax.legend(loc="upper right", fontsize=8)
            fig.tight_layout()
            p = os.path.join(outdir,
                             "fieldsim_run%d_%s_%s.png" % (idx, tag, mtag))
            fig.savefig(p, bbox_inches="tight")
            plt.close(fig)
            written.append(p)
            print("run%d %-5s %-10s: %d scans  %.1f%% LOS  -> %s"
                  % (idx, tag, mtag, len(sc), los, os.path.basename(p)))
    print("\nwrote %d images to %s" % (len(written), outdir))
    return written


if __name__ == "__main__":
    if "--perrun" in sys.argv:
        main_perrun()
    elif "--nruns" in sys.argv:
        main_nruns()
    elif "--gz" in sys.argv:
        main_gz()
    else:
        main()
