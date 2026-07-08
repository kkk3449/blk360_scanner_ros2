#!/usr/bin/env python3
"""Occupancy grid (PGM+YAML) -> USD world with COLLIDABLE walls + ground.

Isaac Sim needs physical colliders for the robot's lidar to hit and for Nav2
to work; a point cloud is not a collider. This mirrors occ_to_world.py (which
builds the Gazebo SDF) but emits a USD stage: occupied cells are merged into
axis-aligned wall boxes, each a Cube with the PhysicsCollisionAPI, plus a
ground plane. Frame = the occupancy-map frame (origin from the YAML), so the
robot's runtime SLAM map is self-consistent.

Usage:
  occ_to_usd_world.py <map.pgm> <map.yaml> [out.usda] [--wall-height 2.5]
"""
import argparse
import sys

import numpy as np


def read_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5"
        d = f.readline()
        while d.startswith(b"#"):
            d = f.readline()
        w, h = map(int, d.split())
        f.readline()
        img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
    return np.flipud(img)            # row0 -> bottom (map frame, y up)


def read_yaml(path):
    res, ox, oy = 0.05, 0.0, 0.0
    for line in open(path):
        s = line.strip()
        if s.startswith("resolution:"):
            res = float(s.split(":")[1])
        if s.startswith("origin:"):
            n = s.split("[")[1].split("]")[0].split(",")
            ox, oy = float(n[0]), float(n[1])
    return res, ox, oy


def merge_rects(occ):
    """Greedy horizontal-run + vertical-span merge -> (i0,i1,j0,j1) cell rects."""
    H, W = occ.shape
    open_rects, done = {}, []
    for j in range(H):
        runs, i = set(), 0
        row = occ[j]
        while i < W:
            if row[i]:
                i0 = i
                while i < W and row[i]:
                    i += 1
                runs.add((i0, i - 1))
            else:
                i += 1
        for span, j0 in list(open_rects.items()):
            if span not in runs:
                done.append((span[0], span[1], j0, j - 1))
                del open_rects[span]
        for span in runs:
            open_rects.setdefault(span, j)
    for span, j0 in open_rects.items():
        done.append((span[0], span[1], j0, H - 1))
    return done


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pgm")
    ap.add_argument("yaml")
    ap.add_argument("out", nargs="?", default=None)
    ap.add_argument("--wall-height", type=float, default=2.5)
    ap.add_argument("--occupied-below", type=int, default=100,
                    help="PGM value below which a cell is a wall (0=black)")
    args = ap.parse_args()

    grid = read_pgm(args.pgm)
    res, ox, oy = read_yaml(args.yaml)
    occ = grid < args.occupied_below
    rects = merge_rects(occ)
    hz = args.wall_height
    out = args.out or args.pgm.replace(".pgm", "_world.usda")

    L = ['#usda 1.0', '(', '    defaultPrim = "World"',
         '    metersPerUnit = 1', '    upAxis = "Z"', ')', '',
         'def Xform "World"', '{',
         '    def PhysicsScene "physicsScene" {}', '',
         '    def Xform "GroundPlane"',
         '    {',
         '        def Mesh "ground" (prepend apiSchemas = ["PhysicsCollisionAPI"])',
         '        {',
         '            int[] faceVertexCounts = [4]',
         '            int[] faceVertexIndices = [0, 1, 2, 3]']
    gx0, gy0 = ox, oy
    gx1, gy1 = ox + grid.shape[1] * res, oy + grid.shape[0] * res
    L.append(f'            point3f[] points = [({gx0},{gy0},0), ({gx1},{gy0},0), '
             f'({gx1},{gy1},0), ({gx0},{gy1},0)]')
    L.append('            color3f[] primvars:displayColor = [(0.7, 0.7, 0.72)]')
    L.append('        }')
    L.append('    }')
    L.append('')
    L.append('    def Xform "Walls"')
    L.append('    {')
    for k, (i0, i1, j0, j1) in enumerate(rects):
        sx = (i1 - i0 + 1) * res
        sy = (j1 - j0 + 1) * res
        cx = ox + (i0 + i1 + 1) * 0.5 * res
        cy = oy + (j0 + j1 + 1) * 0.5 * res
        L.append(f'        def Cube "wall_{k}" '
                 f'(prepend apiSchemas = ["PhysicsCollisionAPI"])')
        L.append('        {')
        L.append('            double size = 1')
        L.append(f'            double3 xformOp:translate = ({cx:.3f}, {cy:.3f}, {hz/2:.3f})')
        L.append(f'            double3 xformOp:scale = ({sx:.3f}, {sy:.3f}, {hz:.3f})')
        L.append('            uniform token[] xformOpOrder = '
                 '["xformOp:translate", "xformOp:scale"]')
        L.append('            color3f[] primvars:displayColor = [(0.55, 0.55, 0.6)]')
        L.append('        }')
    L.append('    }')
    L.append('}')
    with open(out, "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"[usd-world] {occ.sum()} occupied cells -> {len(rects)} collidable "
          f"wall boxes, ground {gx1-gx0:.1f} x {gy1-gy0:.1f} m -> {out}")


if __name__ == "__main__":
    main()
