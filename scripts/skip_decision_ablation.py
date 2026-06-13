#!/usr/bin/env python3
"""Controlled offline ablation: disk vs visibility scan-skip decision.

The live multi-room runs cannot isolate the skip rule because exploration
(Nav2 + frontier) is stochastic: which rooms get mapped dominates the
coverage, independent of the skip decision. This script removes that
confound. On ONE fixed environment map and ONE fixed candidate sequence
(the robot path, sampled every `interval` m), it replays both skip rules
and reports scans taken + final line-of-sight coverage. The candidate
sequence is identical for both rules, so the only thing that differs is
the decision -- a clean apples-to-apples comparison.

  disk       : skip a candidate if it lies within R (straight line) of any
               previous scan -- ignores walls.
  visibility : skip unless the candidate's ray-cast region B(c,R) adds
               >= tau of its own area OR >= A_min m^2 of NEW visible area.

Usage:
  skip_decision_ablation.py <map.pgm> <map.yaml> <traj.csv> [R tau A_min interval]
"""
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "src", "blk360_stop_scan"))
from blk360_stop_scan.visibility import (  # noqa: E402
    new_visible_ratio, union_visible_mask, visible_mask)


def load_map(pgm, yaml):
    with open(pgm, "rb") as f:
        assert f.readline().strip() == b"P5"
        d = f.readline()
        while d.startswith(b"#"):
            d = f.readline()
        w, h = map(int, d.split())
        f.readline()
        img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    g = np.flipud(img).astype(np.int16)               # row0 -> map-frame bottom
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


def candidates_from_traj(traj_csv, interval):
    """Sample candidate poses every `interval` m of path length."""
    pts = []
    for row in csv.reader(open(traj_csv)):
        try:
            pts.append((float(row[0]), float(row[1])))
        except (ValueError, IndexError):
            pass
    cands = []
    if not pts:
        return cands
    cands.append(pts[0])
    acc = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        acc += ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        if acc >= interval:
            cands.append(b)
            acc = 0.0
    return cands


def replay(rule, cands, grid, res, org, R, tau, amin):
    scans = []
    for xy in cands:
        if not scans:
            scans.append(xy)
            continue
        if rule == "disk":
            dmin = min(((xy[0] - s[0]) ** 2 + (xy[1] - s[1]) ** 2) ** 0.5
                       for s in scans)
            if dmin >= R:
                scans.append(xy)
        else:  # visibility
            gain, cand_area, new_area = new_visible_ratio(
                grid, res, org, xy, scans, R)
            if gain is None:
                continue
            if gain >= tau or new_area >= amin:
                scans.append(xy)
    cov = union_visible_mask(grid, res, org, scans, R)
    free = (grid >= 0) & (grid <= 25)
    los = 100.0 * (cov & free).sum() / max(int(free.sum()), 1)
    return scans, los


def main():
    pgm, yaml, traj = sys.argv[1:4]
    R = float(sys.argv[4]) if len(sys.argv) > 4 else 6.0
    tau = float(sys.argv[5]) if len(sys.argv) > 5 else 0.30
    amin = float(sys.argv[6]) if len(sys.argv) > 6 else 5.0
    interval = float(sys.argv[7]) if len(sys.argv) > 7 else 2.0

    grid, res, org = load_map(pgm, yaml)
    free_m2 = ((grid >= 0) & (grid <= 25)).sum() * res * res
    cands = candidates_from_traj(traj, interval)
    print(f"map free {free_m2:.0f} m^2 | {len(cands)} candidates "
          f"(every {interval} m) | R={R} tau={tau} A_min={amin}\n")
    out = {}
    for rule in ["disk", "visibility"]:
        scans, los = replay(rule, cands, grid, res, org, R, tau, amin)
        out[rule] = (scans, los)
        print(f"{rule:11s}: {len(scans)} scans | LOS coverage {los:.1f}%")
        print("   positions:", [[round(x, 2), round(y, 2)] for x, y in scans])
    return out


if __name__ == "__main__":
    main()
