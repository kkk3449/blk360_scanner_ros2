#!/usr/bin/env python3
"""Generic: registered BLK360 scan (.e57/.ply) -> 2D occupancy grid + scene PLY.

Generalizes build_testroom_map.py (which was hard-coded to testroom260601.e57)
for the AMMR digital-twin world: keeps the ORIGINAL e57 XY frame so the
occupancy map, the TOSM semantic USD and the collidable wall USD all align.

Outputs into --outdir:
  map_<name>.pgm / .yaml   nav2 map_server grid (254 free / 0 occ / 205 unknown)
  scene_<name>.ply         voxel-downsampled full cloud (raw frame, walls incl.)
  meta_<name>.npz          floor_z, res, origin, bounds

floor_z is printed; pass it (negated) as build_usd.py --floor-offset and as the
z-shift when placing the semantic overlay in Isaac (raw z + offset -> floor=0).

Run with the blk360_seg venv (pye57/open3d):
  /home/caselab/Downloads/Cyclone360_data/blk360_seg/.venv/bin/python \
      scripts/build_occ_from_e57.py "/path/vis_n2 1.e57" --outdir ~/ammr_twin
"""
import argparse
import os
import sys

import numpy as np

SEG = "/home/caselab/Downloads/Cyclone360_data/blk360_seg"
sys.path.insert(0, SEG)
from blk360seg import io, preprocess  # noqa: E402


def find_floor_z(xyz, max_planes=6):
    """RANSAC the big horizontal planes; the floor is the lowest one."""
    import open3d as o3d
    pts = xyz.copy()
    floor = None
    for _ in range(max_planes):
        if len(pts) < 5000:
            break
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        model, inl = pc.segment_plane(0.05, ransac_n=3, num_iterations=300)
        if len(inl) < 0.03 * len(pts):
            break
        a, b, c, d = model
        nz = abs(c) / (np.sqrt(a * a + b * b + c * c) + 1e-9)
        zmean = pts[inl, 2].mean()
        if nz > 0.9 and (floor is None or zmean < floor):
            floor = zmean
        keep = np.ones(len(pts), bool)
        keep[inl] = False
        pts = pts[keep]
    if floor is None:                       # fallback: low z percentile
        floor = np.percentile(xyz[:, 2], 1.0)
    return float(floor)


def main():
    from scipy import ndimage
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("--outdir", default=os.path.expanduser("~/ammr_twin"))
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--voxel", type=float, default=0.03)
    ap.add_argument("--zband", nargs=2, type=float, default=[0.10, 1.80],
                    help="occupied slice, metres above the floor")
    ap.add_argument("--min-count", type=int, default=3)
    ap.add_argument("--dilate", type=int, default=1,
                    help="wall dilation (cells) for continuity")
    ap.add_argument("--min-blob-m2", type=float, default=0.02,
                    help="drop occupied blobs smaller than this")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    name = os.path.splitext(os.path.basename(args.input))[0].replace(" ", "_")

    print(f"[occ] loading {args.input}")
    xyz, rgb = io.load(args.input)
    print(f"[occ] {len(xyz):,} pts")
    xyz, rgb = preprocess.voxel_downsample(xyz, rgb, args.voxel)
    print(f"[occ] downsampled {len(xyz):,} @ {args.voxel} m")

    floor_z = find_floor_z(xyz)
    print(f"[occ] floor_z = {floor_z:.3f}  (use --floor-offset {-floor_z:.3f})")

    # bounds from robust percentiles (through-window outliers ignored)
    x0, x1 = np.percentile(xyz[:, 0], [0.2, 99.8])
    y0, y1 = np.percentile(xyz[:, 1], [0.2, 99.8])
    pad = 0.3
    x0, x1, y0, y1 = x0 - pad, x1 + pad, y0 + -pad, y1 + pad
    W = int(np.ceil((x1 - x0) / args.res))
    H = int(np.ceil((y1 - y0) / args.res))
    print(f"[occ] grid {W} x {H} @ {args.res} m  origin ({x0:.2f}, {y0:.2f})")

    def raster(pts):
        m = ((pts[:, 0] >= x0) & (pts[:, 0] < x1)
             & (pts[:, 1] >= y0) & (pts[:, 1] < y1))
        q = pts[m]
        ix = ((q[:, 0] - x0) / args.res).astype(int)
        iy = ((q[:, 1] - y0) / args.res).astype(int)
        g = np.zeros((H, W), np.int32)
        np.add.at(g, (iy, ix), 1)
        return g

    zrel = xyz[:, 2] - floor_z
    occ_cnt = raster(xyz[(zrel >= args.zband[0]) & (zrel <= args.zband[1])])
    flr_cnt = raster(xyz[np.abs(zrel) <= 0.06])

    occ = occ_cnt >= args.min_count
    # clean tiny speckle, then close + dilate for continuous walls
    lab, n = ndimage.label(occ)
    sizes = ndimage.sum(occ, lab, range(1, n + 1))
    min_cells = args.min_blob_m2 / (args.res ** 2)
    occ = np.isin(lab, np.where(sizes >= min_cells)[0] + 1)
    occ = ndimage.binary_closing(occ, np.ones((3, 3)))
    if args.dilate > 0:
        occ = ndimage.binary_dilation(occ, iterations=args.dilate)
    free = (flr_cnt >= 1) & ~occ

    grid = np.full((H, W), 205, np.uint8)
    grid[free] = 254
    grid[occ] = 0

    pgm = os.path.join(args.outdir, f"map_{name}.pgm")
    with open(pgm, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (W, H))
        f.write(np.flipud(grid).tobytes())     # map_server: row0 = top (max y)
    yml = os.path.join(args.outdir, f"map_{name}.yaml")
    with open(yml, "w") as f:
        f.write(f"image: map_{name}.pgm\nmode: trinary\nresolution: {args.res}\n"
                f"origin: [{x0:.3f}, {y0:.3f}, 0.0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")

    import open3d as o3d
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pc.colors = o3d.utility.Vector3dVector(rgb.astype(np.float64))
    ply = os.path.join(args.outdir, f"scene_{name}.ply")
    o3d.io.write_point_cloud(ply, pc)

    np.savez(os.path.join(args.outdir, f"meta_{name}.npz"),
             floor_z=floor_z, res=args.res, origin=np.array([x0, y0]),
             occ=occ, free=free)
    print(f"[occ] occupied {int(occ.sum())} cells, free {int(free.sum())} cells")
    print(f"[occ] wrote {pgm}, {yml}, {ply}")


if __name__ == "__main__":
    main()
