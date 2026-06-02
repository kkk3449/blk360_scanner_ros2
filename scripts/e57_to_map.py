#!/usr/bin/env python3
"""Convert a BLK360 e57 point cloud into a 2D occupancy grid (floor plan).

Strategy:
  1. Read XYZ from the scan.
  2. Auto-detect the floor height (densest low horizontal band).
  3. Keep points in a wall-height band [floor+band_lo, floor+band_hi] so the
     floor and ceiling are excluded and only vertical structure (walls,
     furniture) projects down.
  4. Accumulate those points into an XY grid; cells with >= min_pts are walls.
  5. Write a ROS map_server-style occupancy grid: <name>.pgm + <name>.yaml,
     plus a <name>_preview.png for a quick human look.

Usage:
  e57_to_map.py <in.e57> <out_dir> <name> [--res 0.05] [--band 0.4 1.8]
                [--min-pts 3] [--clip 0.5 99.5]
"""
import argparse
import os

import numpy as np
import pye57


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("e57")
    ap.add_argument("out_dir")
    ap.add_argument("name")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--band", type=float, nargs=2, default=[0.4, 1.8],
                    help="wall band lo/hi above detected floor (m)")
    ap.add_argument("--min-pts", type=int, default=3,
                    help="min points per cell to mark occupied")
    ap.add_argument("--clip", type=float, nargs=2, default=[0.5, 99.5],
                    help="XY percentile clip to drop far outliers")
    ap.add_argument("--close", type=int, default=1,
                    help="morphological closing iterations to connect walls")
    ap.add_argument("--despeckle", type=int, default=3,
                    help="drop occupied cells with fewer than N occupied 8-neighbours")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    print(f"[map] reading {args.e57} ...")
    e = pye57.E57(args.e57)
    d = e.read_scan(0, ignore_missing_fields=True, intensity=False, colors=False)
    x = np.asarray(d["cartesianX"], dtype=np.float64)
    y = np.asarray(d["cartesianY"], dtype=np.float64)
    z = np.asarray(d["cartesianZ"], dtype=np.float64)
    m = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[m], y[m], z[m]
    print(f"[map] {x.size} finite points")

    # --- floor detection: densest 0.1 m z-bin in the lower 40% of the z range ---
    zlo, zhi = np.percentile(z, 1), np.percentile(z, 99)
    lz = z[(z >= zlo) & (z <= zhi)]
    bins = np.arange(lz.min(), lz.max() + 0.1, 0.1)
    hist, edges = np.histogram(lz, bins=bins)
    lower_region = edges[:-1] < (zlo + 0.4 * (zhi - zlo))
    cand = np.where(lower_region, hist, -1)
    floor_z = edges[:-1][int(np.argmax(cand))] + 0.05
    print(f"[map] z range [{zlo:.2f},{zhi:.2f}], detected floor_z ~ {floor_z:.2f}")

    band_lo, band_hi = floor_z + args.band[0], floor_z + args.band[1]
    wm = (z >= band_lo) & (z <= band_hi)
    wx, wy = x[wm], y[wm]
    print(f"[map] wall band [{band_lo:.2f},{band_hi:.2f}] -> {wx.size} points")

    # --- XY bounds via percentile clip (drop far reflections/outliers) ---
    xmin, xmax = np.percentile(wx, args.clip[0]), np.percentile(wx, args.clip[1])
    ymin, ymax = np.percentile(wy, args.clip[0]), np.percentile(wy, args.clip[1])
    inb = (wx >= xmin) & (wx <= xmax) & (wy >= ymin) & (wy <= ymax)
    wx, wy = wx[inb], wy[inb]
    pad = 0.5
    xmin, xmax, ymin, ymax = xmin - pad, xmax + pad, ymin - pad, ymax + pad
    W = int(np.ceil((xmax - xmin) / args.res))
    H = int(np.ceil((ymax - ymin) / args.res))
    print(f"[map] grid {W} x {H} cells @ {args.res} m  (extent {xmax-xmin:.1f} x {ymax-ymin:.1f} m)")

    # --- accumulate point counts per cell ---
    ci = ((wx - xmin) / args.res).astype(np.int32)
    cj = ((wy - ymin) / args.res).astype(np.int32)
    np.clip(ci, 0, W - 1, out=ci)
    np.clip(cj, 0, H - 1, out=cj)
    counts = np.zeros((H, W), dtype=np.int32)
    np.add.at(counts, (cj, ci), 1)
    occ = counts >= args.min_pts

    # --- despeckle: drop isolated occupied cells (2D radius-outlier analogue,
    #     inspired by pcd2pgm's RadiusOutlierRemoval) ---
    if args.despeckle > 0:
        occ = _despeckle(occ, args.despeckle)
        print(f"[map] after despeckle (min {args.despeckle} nbrs): {int(occ.sum())} cells")

    # --- optional morphological closing (connect wall gaps) ---
    if args.close > 0:
        occ = _binary_close(occ, args.close)
    print(f"[map] occupied cells: {int(occ.sum())}")

    # --- write PGM (ROS map convention: 0=occupied black, 254=free, 205=unknown) ---
    # Floor plan: inside-bounds free, walls occupied. No 'unknown' for a clean world.
    img = np.full((H, W), 254, dtype=np.uint8)
    img[occ] = 0
    # PGM origin is bottom-left in ROS, but image row 0 is top -> flip vertically.
    img_pgm = np.flipud(img)
    pgm_path = os.path.join(args.out_dir, args.name + ".pgm")
    _write_pgm(pgm_path, img_pgm)
    print(f"[map] wrote {pgm_path}")

    yaml_path = os.path.join(args.out_dir, args.name + ".yaml")
    with open(yaml_path, "w") as f:
        f.write(f"image: {args.name}.pgm\n")
        f.write(f"resolution: {args.res}\n")
        f.write(f"origin: [{xmin:.4f}, {ymin:.4f}, 0.0]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    print(f"[map] wrote {yaml_path}")

    # --- save metadata for the world generator ---
    meta_path = os.path.join(args.out_dir, args.name + "_meta.npz")
    np.savez(meta_path, occ=occ, res=args.res, xmin=xmin, ymin=ymin,
             floor_z=floor_z)
    print(f"[map] wrote {meta_path}")

    # --- preview PNG (best-effort) ---
    try:
        _write_png_preview(os.path.join(args.out_dir, args.name + "_preview.png"),
                           img_pgm)
        print("[map] wrote preview png")
    except Exception as ex:
        print(f"[map] preview skipped: {ex}")
    print("[map] MAP_DONE")


def _despeckle(a, min_nbrs):
    # Count 8-neighbours for each occupied cell; drop those below min_nbrs.
    a = a.astype(np.uint8)
    nbr = np.zeros_like(a, dtype=np.int32)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nbr[max(dy, 0):a.shape[0] + min(dy, 0),
                max(dx, 0):a.shape[1] + min(dx, 0)] += \
                a[max(-dy, 0):a.shape[0] + min(-dy, 0),
                  max(-dx, 0):a.shape[1] + min(-dx, 0)]
    return (a.astype(bool)) & (nbr >= min_nbrs)


def _binary_close(a, it):
    # simple dilation then erosion using 4-neighbour shifts, numpy-only
    def dil(b):
        out = b.copy()
        out[1:, :] |= b[:-1, :]; out[:-1, :] |= b[1:, :]
        out[:, 1:] |= b[:, :-1]; out[:, :-1] |= b[:, 1:]
        return out
    def ero(b):
        out = b.copy()
        out[1:, :] &= b[:-1, :]; out[:-1, :] &= b[1:, :]
        out[:, 1:] &= b[:, :-1]; out[:, :-1] &= b[:, 1:]
        return out
    for _ in range(it):
        a = dil(a)
    for _ in range(it):
        a = ero(a)
    return a


def _write_pgm(path, img):
    H, W = img.shape
    with open(path, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        f.write(img.tobytes())


def _write_png_preview(path, img):
    # Use PIL if available; otherwise skip.
    from PIL import Image
    Image.fromarray(img, mode="L").save(path)


if __name__ == "__main__":
    main()
