#!/usr/bin/env python3
"""Controlled 3-way skip-rule ablation: uniform vs disk vs visibility.

Extends skip_decision_ablation.py to a third baseline and adds a
distance-traveled metric, all evaluated under the SAME line-of-sight (LOS)
coverage criterion on the SAME N fixed candidate paths per environment, so
the three rows are directly comparable.

Rules (all replayed on one identical candidate path):
  uniform    : accept EVERY candidate -- the ICCAS distance-traveled trigger
               (scan every d m, no skipping). Many scans, near-full LOS.
  disk       : skip a candidate within R (straight line) of any prior scan.
               Few scans, low LOS (over-skips behind walls).
  visibility : accept unless B(c,R) adds < tau of its own area AND < A_min m^2
               of NEW visible area. Moderate scans, high LOS.

Distance-traveled is reported two ways:
  to_last_scan : path length from start to the last ACCEPTED scan pose
                 (rule-dependent: the travel the policy commits to before it
                 stops scanning).
  full_path    : total candidate-path length (rule-independent, for context).

The disk/visibility numbers reproduce the existing Table; the uniform row and
the distance columns are the new additions.

Usage:
  skip_rule_3way.py            # runs both environments, prints the table
"""
import os
import sys

import numpy as np

HOME = os.path.expanduser("~")
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "src", "blk360_stop_scan"))
from blk360_stop_scan.visibility import (  # noqa: E402
    new_visible_ratio, union_visible_mask)


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
    return grid, res, (ox, oy)


def traj_xy(path):
    xs, ys = [], []
    for line in open(path):
        p = line.strip().split(",")
        try:
            xs.append(float(p[0]))
            ys.append(float(p[1]))
        except (ValueError, IndexError):
            pass
    return xs, ys


def candidate_path(traj_csv, interval=2.0):
    """Sample candidates every `interval` m; also return the cumulative path
    length (from the full-resolution trajectory) at each candidate and the
    total path length. Matches render_journal_figs.candidate_path sampling."""
    tx, ty = traj_xy(traj_csv)
    if not tx:
        return [], [], 0.0
    cands = [(tx[0], ty[0])]
    cum_at_cand = [0.0]
    acc, total = 0.0, 0.0
    for i in range(1, len(tx)):
        step = ((tx[i] - tx[i - 1]) ** 2 + (ty[i] - ty[i - 1]) ** 2) ** 0.5
        acc += step
        total += step
        if acc >= interval:
            cands.append((tx[i], ty[i]))
            cum_at_cand.append(total)
            acc = 0.0
    return cands, cum_at_cand, total


def load_env(mapdir, names):
    """Most-complete reference map + candidate paths (with arc-lengths)."""
    best, maps = None, {}
    for nm in names:
        try:
            grid, res, org = load_map(f"{mapdir}/map_{nm}.pgm",
                                      f"{mapdir}/map_{nm}.yaml")
        except FileNotFoundError:
            continue
        maps[nm] = (grid, res, org)
        fa = ((grid >= 0) & (grid <= 25)).sum()
        if best is None or fa > best[0]:
            best = (fa, nm)
    grid, res, org = maps[best[1]]
    free = (grid >= 0) & (grid <= 25)
    paths = []
    for nm in names:
        f = f"{mapdir}/traj_{nm}.csv"
        if os.path.exists(f):
            c, cum, total = candidate_path(f)
            if len(c) >= 3:
                paths.append((c, cum, total))
    return grid, res, org, free, paths


def replay(rule, cands, cum, grid, res, org, free, R=6.0, tau=0.30, amin=5.0):
    """Return (n_scans, LOS%, dist_to_last_scan_m)."""
    scans, last_idx = [], 0
    for i, xy in enumerate(cands):
        if not scans:
            scans.append(xy)
            last_idx = i
            continue
        if rule == "uniform":
            scans.append(xy)
            last_idx = i
        elif rule == "disk":
            dmin = min(((xy[0] - s[0]) ** 2 + (xy[1] - s[1]) ** 2) ** 0.5
                       for s in scans)
            if dmin >= R:
                scans.append(xy)
                last_idx = i
        else:  # visibility
            g, _, na = new_visible_ratio(grid, res, org, xy, scans, R)
            if g is None:
                continue
            if g >= tau or na >= amin:
                scans.append(xy)
                last_idx = i
    cov = union_visible_mask(grid, res, org, scans, R)
    los = 100.0 * (cov & free).sum() / max(int(free.sum()), 1)
    return len(scans), los, cum[last_idx]


def summarize(env_name, mapdir, names, R=6.0, tau=0.30, amin=5.0):
    grid, res, org, free, paths = load_env(mapdir, names)
    free_m2 = int(free.sum()) * res * res
    rows = {}
    for rule in ["uniform", "disk", "visibility"]:
        scans, los, dlast = [], [], []
        for c, cum, total in paths:
            n, l, d = replay(rule, c, cum, grid, res, org, free, R, tau, amin)
            scans.append(n)
            los.append(l)
            dlast.append(d)
        sd = lambda v: float(np.std(v, ddof=1)) if len(v) > 1 else 0.0  # sample
        spacing = [t / max(n, 1) for n, (_, _, t) in zip(scans, paths)]
        rows[rule] = dict(
            scans=(np.mean(scans), sd(scans)),
            los=(np.mean(los), sd(los)),
            dist=(np.mean(dlast), sd(dlast)),
            spacing=(np.mean(spacing), sd(spacing)),
            raw_scans=scans, raw_los=los, raw_dist=dlast)
    full = [total for _, _, total in paths]
    return dict(name=env_name, n=len(paths), free_m2=free_m2,
                full_path=(np.mean(full), np.std(full)), rows=rows)


def fmt(env):
    print(f"\n=== {env['name']}  (N={env['n']} paths, "
          f"free={env['free_m2']:.0f} m^2, "
          f"full path {env['full_path'][0]:.1f}+/-{env['full_path'][1]:.1f} m) ===")
    print(f"{'rule':12s} {'scans':>11s} {'LOS %':>13s} "
          f"{'dist-last(m)':>15s} {'m/scan':>13s}")
    for rule in ["uniform", "disk", "visibility"]:
        r = env["rows"][rule]
        print(f"{rule:12s} "
              f"{r['scans'][0]:5.1f}+/-{r['scans'][1]:<4.1f} "
              f"{r['los'][0]:6.1f}+/-{r['los'][1]:<5.1f} "
              f"{r['dist'][0]:7.1f}+/-{r['dist'][1]:<5.1f} "
              f"{r['spacing'][0]:6.1f}+/-{r['spacing'][1]:<5.1f}")


def main():
    envs = [
        summarize("Single-room", f"{HOME}/blk360_sotastats",
                  [f"sota_run{i}" for i in range(1, 6)]),
        summarize("Multi-room", f"{HOME}/blk360_mrstats",
                  [f"mr_{m}_run{i}" for m in ["disk", "visibility"]
                   for i in range(1, 6)]),
    ]
    for e in envs:
        fmt(e)
    return envs


if __name__ == "__main__":
    main()
