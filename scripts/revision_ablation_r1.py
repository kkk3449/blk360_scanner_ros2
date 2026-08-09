#!/usr/bin/env python3
"""Sensors major-revision analysis (R1.7 / R2.1 / R2.5).

Replays FOUR placement policies on the identical fixed candidate paths used by
the submitted controlled ablation (N=5 single-room, N=10 multi-room):

  uniform    : scan at every candidate (no-skip upper reference)
  disk_space : skip if within R (straight line) of a prior scan
               (the de facto proximity-spacing rule of stop-and-go practice)
  disk_gain  : NEW -- the SAME dual-criterion marginal-gain rule as the
               visibility policy, but computed on the isotropic-disk coverage
               model B_disk (free cells within R, no occlusion reasoning)
  visibility : ray-cast visibility model + dual-criterion rule (ours)

All policies are evaluated by ray-cast LOS coverage on the common reference
map, so rows differ only in the accept/skip decision. Writes per-path records
and mean+-sd to outputs_thesis/journal/revision_ablation.json.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "blk360_stop_scan"))
from blk360_stop_scan.visibility import (  # noqa: E402
    new_visible_ratio, union_visible_mask)

HOME = os.path.expanduser("~")
OUT = os.path.join(HOME, "blk360_ros2_ws", "outputs_thesis", "journal")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_journal_figs import load_env  # noqa: E402


def disk_mask(grid, res, org, x, y, R):
    """Isotropic-disk coverage B_disk: mapped-free cells within R of (x, y)."""
    H, W = grid.shape
    free = (grid >= 0) & (grid <= 25)
    rows, cols = np.mgrid[0:H, 0:W]
    cx = org[0] + (cols + 0.5) * res
    cy = org[1] + (rows + 0.5) * res
    return free & ((cx - x) ** 2 + (cy - y) ** 2 <= R * R)


def replay(policy, cands, grid, res, org, free, R=6.0, tau=0.30, amin=5.0):
    sc = []
    cov_disk = np.zeros_like(free)          # disk-model coverage bookkeeping
    for xy in cands:
        if not sc:
            sc.append(xy)
            if policy == "disk_gain":
                cov_disk |= disk_mask(grid, res, org, *xy, R)
            continue
        if policy == "uniform":
            sc.append(xy)
        elif policy == "disk_space":
            if min(((xy[0] - s[0]) ** 2 + (xy[1] - s[1]) ** 2) ** 0.5
                   for s in sc) >= R:
                sc.append(xy)
        elif policy == "disk_gain":
            b = disk_mask(grid, res, org, *xy, R)
            tot = int(b.sum())
            if tot * res * res < 0.5:       # same degenerate cut
                continue
            new = int((b & ~cov_disk).sum())
            g = new / tot
            a = new * res * res
            if g >= tau or a >= amin:
                sc.append(xy)
                cov_disk |= b
        else:                               # visibility
            g, _, na = new_visible_ratio(grid, res, org, xy, sc, R)
            if g is None:
                continue
            if g >= tau or na >= amin:
                sc.append(xy)
    cov = union_visible_mask(grid, res, org, sc, R)
    los = 100.0 * (cov & free).sum() / max(int(free.sum()), 1)
    return len(sc), los


def main():
    envs = {
        "single": load_env(f"{HOME}/blk360_sotastats",
                           [f"sota_run{i}" for i in range(1, 6)]),
        "multi": load_env(f"{HOME}/blk360_mrstats",
                          [f"mr_{m}_run{i}" for m in ["disk", "visibility"]
                           for i in range(1, 6)]),
    }
    policies = ["uniform", "disk_space", "disk_gain", "visibility"]
    out = {}
    for envname, (grid, res, org, free, paths) in envs.items():
        out[envname] = {}
        for pol in policies:
            recs = [replay(pol, c, grid, res, org, free) for c in paths]
            ns = [r[0] for r in recs]
            ls = [r[1] for r in recs]
            out[envname][pol] = {
                "per_path": recs,
                "scans_mean": float(np.mean(ns)),
                "scans_sd": float(np.std(ns, ddof=1)),
                "los_mean": float(np.mean(ls)),
                "los_sd": float(np.std(ls, ddof=1)),
            }
            print(f"{envname:6s} {pol:10s}: "
                  f"scans {np.mean(ns):4.1f}+-{np.std(ns, ddof=1):3.1f} | "
                  f"LOS {np.mean(ls):5.1f}+-{np.std(ls, ddof=1):4.1f}  "
                  f"(n={len(recs)})")
    path = os.path.join(OUT, "revision_ablation.json")
    json.dump(out, open(path, "w"), indent=2)
    print("wrote", path)


if __name__ == "__main__":
    main()
