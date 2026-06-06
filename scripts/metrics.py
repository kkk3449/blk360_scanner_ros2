#!/usr/bin/env python3
"""Aggregate BLK360 active-mapping run data into paper-ready metrics.

Reads the per-run JSON records the stop-scan sequencer writes on completion
(``~/blk360_runs/run_*.json``: scan positions, counts, timing, the coverage
radius R used) and computes scan-placement metrics:

  - scans / skipped / completion time
  - nearest-neighbour scan separation (min / mean / max)
  - covered area  A_cov = area( U_i B(s_i, R) )      (union of coverage disks)
  - total disk area  A_sum = n * pi * R^2
  - overlap ratio  = 1 - A_cov / A_sum               (fraction of redundant cover)
  - area covered per scan  = A_cov / n               (efficiency)
  - room coverage ratio = A_cov / A_room             (A_room = GT free area)

The separation / area / overlap metrics are frame-invariant (they only use
relative scan geometry), so no map<->GT alignment is needed. `room coverage`
uses the GT free *area* as a scalar room size.

Outputs a CSV table (one row per run) and, if matplotlib is available, a figure
per run showing the scan centres + coverage disks.

Usage:
  scripts/metrics.py [--runs-dir ~/blk360_runs] [--gt <testroom_gt.yaml>]
                     [--out <dir>] [--summary-log ~/blk360_exploration_summary.log]
"""
import argparse
import csv
import glob
import json
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
DEFAULT_GT = os.path.join(WS, "src", "blk360_bringup", "maps", "testroom_gt.yaml")


def read_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5", "not a P5 pgm"
        dims = f.readline()
        while dims.startswith(b"#"):
            dims = f.readline()
        w, h = map(int, dims.split())
        f.readline()  # maxval
        data = np.frombuffer(f.read(), dtype=np.uint8)
    return data.reshape(h, w)


def load_gt(yaml_path):
    """Load a ROS map yaml+pgm. Returns dict with free-cell world coords (Fx,Fy),
    resolution and total free area (254 = free in our GT convention)."""
    res, xmin, ymin, img_name = 0.05, 0.0, 0.0, None
    for line in open(yaml_path):
        line = line.strip()
        if line.startswith("resolution:"):
            res = float(line.split(":")[1])
        elif line.startswith("image:"):
            img_name = line.split(":", 1)[1].strip()
        elif line.startswith("origin:"):
            nums = line.split("[")[1].split("]")[0].split(",")
            xmin, ymin = float(nums[0]), float(nums[1])
    img = read_pgm(os.path.join(os.path.dirname(yaml_path), img_name))
    H, W = img.shape
    free = img >= 250
    occ = img <= 50
    rr, cc = np.where(free)                # image rows (0=top) / cols
    # ROS map: row 0 of the pgm is the TOP (highest y) due to the vertical flip.
    fx = xmin + (cc + 0.5) * res
    fy = ymin + (H - 1 - rr + 0.5) * res
    return {"Fx": fx, "Fy": fy, "res": res,
            "free_mask": free, "occ_mask": occ,
            "free_area": float(free.sum()) * res * res}


def _read_map(yaml_path):
    res, img_name = 0.05, None
    for line in open(yaml_path):
        line = line.strip()
        if line.startswith("resolution:"):
            res = float(line.split(":")[1])
        elif line.startswith("image:"):
            img_name = line.split(":", 1)[1].strip()
    img = read_pgm(os.path.join(os.path.dirname(yaml_path), img_name))
    return img, res


def _best_shift(occ_a, occ_b):
    """Integer (dy, dx) translation so occ_b aligned onto occ_a (cross-corr peak).
    Assumes ~no rotation (true for Cartographer started axis-aligned in sim)."""
    from scipy.signal import fftconvolve
    corr = fftconvolve(occ_a.astype(float), occ_b[::-1, ::-1].astype(float), mode="full")
    pk = np.unravel_index(int(np.argmax(corr)), corr.shape)
    return pk[0] - (occ_b.shape[0] - 1), pk[1] - (occ_b.shape[1] - 1)


def _apply_shift(mask, dy, dx, shape):
    out = np.zeros(shape, dtype=bool)
    OH, OW = shape
    m0, m1 = max(0, dy), min(OH, mask.shape[0] + dy)
    n0, n1 = max(0, dx), min(OW, mask.shape[1] + dx)
    if m1 > m0 and n1 > n0:
        out[m0:m1, n0:n1] = mask[m0 - dy:m1 - dy, n0 - dx:n1 - dx]
    return out


def map_quality(slam_yaml, gt):
    """Compare a saved SLAM map to the GT map after translation alignment.
    Returns free-space IoU, GT-free coverage %, and wall RMSE (m)."""
    from scipy.ndimage import distance_transform_edt
    img, res = _read_map(slam_yaml)
    free_s_raw = img >= 250
    occ_s_raw = img <= 50
    free_g, occ_g = gt["free_mask"], gt["occ_mask"]
    shape = free_g.shape
    dy, dx = _best_shift(occ_g, occ_s_raw)
    free_s = _apply_shift(free_s_raw, dy, dx, shape)
    occ_s = _apply_shift(occ_s_raw, dy, dx, shape)
    inter = np.logical_and(free_g, free_s).sum()
    union = np.logical_or(free_g, free_s).sum()
    iou = float(inter) / float(union) if union else 0.0
    coverage = float(np.logical_and(free_g, free_s).sum()) / float(free_g.sum()) \
        if free_g.sum() else 0.0
    # wall RMSE: distance from each GT wall cell to the nearest SLAM wall cell.
    if occ_s.any() and occ_g.any():
        dist = distance_transform_edt(~occ_s) * gt["res"]
        rmse = float(np.sqrt(np.mean(dist[occ_g] ** 2)))
    else:
        rmse = float("nan")
    return {"iou_free": round(iou, 3),
            "gt_coverage_pct": round(100.0 * coverage, 1),
            "wall_rmse_m": round(rmse, 3)}


def room_coverage_aligned(positions, R, gt):
    """Fraction of GT free area within R of a scan, after centroid-aligning the
    scan positions (map frame) onto the GT free region (world frame). Heuristic
    alignment -> approximate, but spatially meaningful (unlike A_cov/A_room)."""
    if not positions or gt is None or gt["Fx"].size == 0:
        return 0.0
    Fx, Fy = gt["Fx"], gt["Fy"]
    pc = np.array(positions, dtype=float)
    off_x = Fx.mean() - pc[:, 0].mean()
    off_y = Fy.mean() - pc[:, 1].mean()
    covered = np.zeros(Fx.shape, dtype=bool)
    R2 = R * R
    for (px, py) in positions:
        covered |= (Fx - (px + off_x)) ** 2 + (Fy - (py + off_y)) ** 2 <= R2
    return float(covered.mean())


def union_disk_area(positions, R, res=0.05):
    if not positions:
        return 0.0
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    x0, y0 = min(xs) - R, min(ys) - R
    x1, y1 = max(xs) + R, max(ys) + R
    W = int(math.ceil((x1 - x0) / res)) + 1
    H = int(math.ceil((y1 - y0) / res)) + 1
    gx = x0 + np.arange(W) * res
    gy = y0 + np.arange(H) * res
    GX, GY = np.meshgrid(gx, gy)
    covered = np.zeros((H, W), dtype=bool)
    R2 = R * R
    for (px, py) in positions:
        covered |= (GX - px) ** 2 + (GY - py) ** 2 <= R2
    return float(covered.sum()) * res * res


def nn_separations(positions):
    n = len(positions)
    if n < 2:
        return []
    out = []
    for i in range(n):
        out.append(min(math.dist(positions[i], positions[j])
                       for j in range(n) if j != i))
    return out


def run_metrics(rec, gt, slam_yaml=None):
    pos = [tuple(p) for p in rec.get("scan_positions", [])]
    R = float(rec.get("scan_coverage_radius_m", 4.0))
    n = len(pos)
    a_cov = union_disk_area(pos, R)
    a_sum = n * math.pi * R * R
    seps = nn_separations(pos)
    room_cov = room_coverage_aligned(pos, R, gt)
    out = {
        "config": rec.get("config_name", ""),
        "timestamp": rec.get("timestamp", ""),
        "reason": rec.get("reason", ""),
        "scans": n,
        "skipped": rec.get("scans_skipped", 0),
        "completion_s": rec.get("completion_time_s", 0.0),
        "R_m": R,
        "interval_m": rec.get("scan_interval_m", 0.0),
        "suppression": rec.get("frontier_suppression_enabled", ""),
        "nn_sep_min_m": round(min(seps), 2) if seps else 0.0,
        "nn_sep_mean_m": round(sum(seps) / len(seps), 2) if seps else 0.0,
        "nn_sep_max_m": round(max(seps), 2) if seps else 0.0,
        "A_cov_m2": round(a_cov, 1),
        "A_sum_m2": round(a_sum, 1),
        "overlap_ratio": round(1.0 - a_cov / a_sum, 3) if a_sum > 0 else 0.0,
        "area_per_scan_m2": round(a_cov / n, 1) if n else 0.0,
        "room_coverage_pct": round(100.0 * room_cov, 1),
        "iou_free": "", "gt_coverage_pct": "", "wall_rmse_m": "",
    }
    if slam_yaml and os.path.exists(slam_yaml) and gt is not None:
        try:
            out.update(map_quality(slam_yaml, gt))
        except Exception as exc:  # noqa: BLE001
            print(f"[metrics] map quality failed for {os.path.basename(slam_yaml)}: {exc}")
    return out


def plot_run(rec, metrics, out_png):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        import colorsys
    except Exception:
        return False
    pos = [tuple(p) for p in rec.get("scan_positions", [])]
    if not pos:
        return False
    R = float(rec.get("scan_coverage_radius_m", 4.0))
    fig, ax = plt.subplots(figsize=(8, 5))
    for i, (x, y) in enumerate(pos):
        r, g, b = colorsys.hsv_to_rgb((i * 0.6180339887) % 1.0, 0.85, 0.95)
        ax.add_patch(Circle((x, y), R, color=(r, g, b), alpha=0.18))
        ax.plot(x, y, "o", color=(r, g, b), ms=8)
        ax.annotate(f"#{i + 1}", (x, y), fontsize=9, ha="center", va="center")
    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.set_xlabel("x [m] (map frame)")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Scan coverage  | {metrics['scans']} scans, "
                 f"{metrics['skipped']} skipped, overlap={metrics['overlap_ratio']}, "
                 f"room_cov={metrics['room_coverage_pct']}%")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    return True


def parse_summary_log(path):
    """Best-effort list of legacy runs (no positions) from the text summary log."""
    runs = []
    if not os.path.exists(path):
        return runs
    cur = {}
    for line in open(path):
        s = line.strip()
        if "SUMMARY" in s:
            if cur:
                runs.append(cur)
            cur = {"reason": s.split("(")[-1].rstrip(")") if "(" in s else ""}
        elif s.startswith("Scan-completion time"):
            cur["completion_s"] = float(s.split(":")[1].split("s")[0])
        elif s.startswith("BLK360 scans"):
            cur["scans"] = int(s.split(":")[1])
        elif s.startswith("Scans skipped"):
            cur["skipped"] = int(s.split(":")[1])
    if cur:
        runs.append(cur)
    return runs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs-dir", default=os.path.expanduser("~/blk360_runs"))
    ap.add_argument("--gt", default=DEFAULT_GT)
    ap.add_argument("--out", default=os.path.expanduser("~/blk360_runs/metrics"))
    ap.add_argument("--summary-log",
                    default=os.path.expanduser("~/blk360_exploration_summary.log"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    gt = None
    if os.path.exists(args.gt):
        gt = load_gt(args.gt)
        print(f"[metrics] GT free (room) area = {gt['free_area']:.1f} m^2")
    else:
        print(f"[metrics] GT not found at {args.gt}; room_coverage will be 0")

    files = sorted(glob.glob(os.path.join(args.runs_dir, "run_*.json")))
    rows = []
    for fp in files:
        rec = json.load(open(fp))
        base = os.path.basename(fp)
        # sibling SLAM map saved by ablation.sh: run_<name>.json -> map_<name>.yaml
        map_yaml = os.path.join(args.runs_dir,
                                base.replace("run_", "map_").replace(".json", ".yaml"))
        m = run_metrics(rec, gt, map_yaml if os.path.exists(map_yaml) else None)
        m["file"] = base
        rows.append(m)
        png = os.path.join(args.out, base.replace(".json", ".png"))
        if plot_run(rec, m, png):
            print(f"[metrics] figure -> {png}")

    if rows:
        cols = ["config", "file", "timestamp", "reason", "scans", "skipped",
                "completion_s", "R_m", "interval_m", "suppression",
                "nn_sep_min_m", "nn_sep_mean_m", "nn_sep_max_m",
                "A_cov_m2", "A_sum_m2", "overlap_ratio", "area_per_scan_m2",
                "room_coverage_pct", "iou_free", "gt_coverage_pct", "wall_rmse_m"]
        csv_path = os.path.join(args.out, "runs_metrics.csv")
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            for r in rows:
                w.writerow({k: r.get(k, "") for k in cols})
        print(f"[metrics] wrote {csv_path}  ({len(rows)} runs with positions)")
        print()
        hdr = ["config", "scans", "skipped", "completion_s", "R_m", "suppression",
               "overlap_ratio", "room_coverage_pct", "iou_free", "wall_rmse_m"]
        print("  " + "  ".join(f"{h:>13}" for h in hdr))
        for r in rows:
            print("  " + "  ".join(f"{str(r.get(h, '')):>13}" for h in hdr))
    else:
        print(f"[metrics] no run_*.json in {args.runs_dir} yet "
              "(a completed run with the updated sequencer creates one).")

    legacy = parse_summary_log(args.summary_log)
    if legacy:
        print(f"\n[metrics] {len(legacy)} run(s) in the text summary log "
              "(time/scans/skips only, no positions):")
        for r in legacy:
            print(f"    scans={r.get('scans','?')} skipped={r.get('skipped','?')} "
                  f"completion_s={r.get('completion_s','?')} ({r.get('reason','')})")


if __name__ == "__main__":
    main()
