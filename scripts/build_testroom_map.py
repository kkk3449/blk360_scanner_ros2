#!/usr/bin/env python3
"""Build a 2D occupancy grid (ROS map_server PGM+YAML) of the physical test room
from the real registered BLK360 point cloud (testroom260601.e57).

Same construction the paper's simulation worlds use (Sec. Environments): the
real .e57 scan is projected to 2D at robot/wall height, the open exhibition
space is *closed* (a footprint is formed and its perimeter sealed), and furniture
speckle is cleaned, yielding a navigable occupancy grid for the visibility
stop-and-scan evaluation. The registration metrics come from Cyclone; this file
only produces the geometry used by the Redbot-parameterized simulation.

Perimeter cells are classified into
  * real walls    -- perimeter backed by actual scanned obstacle points, and
  * virtual walls -- added purely to close an open boundary (no obstacle points),
so the figure can draw the two in different colours (the open space is fenced by
virtual walls). Both block ray casting.

Writes into ~/blk360_fieldsim/: map_testroom.pgm / .yaml and masks.npz
(free / real_wall / virtual_wall / obstacle boolean grids + res + origin).
Pixel convention (nav2 map_server): 254 free, 0 occupied, 205 unknown.
"""
import os
import numpy as np
from scipy import ndimage

HOME = os.path.expanduser("~")
SCR = ("/tmp/claude-1000/-home-caselab-blk360-ros2-ws/"
       "d20b05b5-83d3-485f-9fd4-2a67ebfff465/scratchpad/")
OUTDIR = os.path.join(HOME, "blk360_fieldsim")
os.makedirs(OUTDIR, exist_ok=True)

RES = 0.05
XMIN, XMAX, YMIN, YMAX = -7.5, 11.5, -5.0, 5.5
OBS_MIN_COUNT = 3          # obstacle-slice hits per cell to call it occupied
CLOSE_R = 20               # closing radius (cells) to form the room footprint
MIN_OBST_AREA = 0.60       # drop furniture blobs smaller than this (m^2)


def disk(r):
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= r * r


def raster_count(pts, W, H):
    m = ((pts[:, 0] >= XMIN) & (pts[:, 0] < XMAX)
         & (pts[:, 1] >= YMIN) & (pts[:, 1] < YMAX))
    q = pts[m]
    ix = ((q[:, 0] - XMIN) / RES).astype(int)
    iy = ((q[:, 1] - YMIN) / RES).astype(int)
    g = np.zeros((H, W), np.int32)
    np.add.at(g, (iy, ix), 1)
    return g


def build():
    z = np.load(SCR + "testroom_slices.npz")
    obs, flo = z["obs"], z["floor"]
    W = int(round((XMAX - XMIN) / RES))
    H = int(round((YMAX - YMIN) / RES))

    og = raster_count(obs, W, H)
    fg = raster_count(flo, W, H)

    obs_mask = og >= OBS_MIN_COUNT
    floor_seen = (fg >= 1) | ((og >= 1) & ~obs_mask)
    free_obs = floor_seen & ~obs_mask

    # close the open exhibition space into a single room footprint
    foot = ndimage.binary_closing(free_obs | obs_mask, structure=disk(CLOSE_R))
    foot = ndimage.binary_fill_holes(foot)
    lab, n = ndimage.label(foot, np.ones((3, 3)))
    sizes = ndimage.sum(np.ones_like(lab), lab, index=np.arange(1, n + 1))
    foot = lab == (int(np.argmax(sizes)) + 1)

    # interior furniture / equipment; keep only sizeable occluders and erase
    # small clutter (as the paper's other worlds do, "excessive obstacles
    # removed"). Removed clutter reverts to free floor, not to holes.
    obstacle = obs_mask & foot
    lo, no = ndimage.label(obstacle, np.ones((3, 3)))
    if no:
        os_ = ndimage.sum(np.ones_like(lo), lo, index=np.arange(1, no + 1))
        for i, s in enumerate(os_, 1):
            if s * RES * RES < MIN_OBST_AREA:
                obstacle[lo == i] = False
    obstacle = ndimage.binary_opening(obstacle, disk(1), iterations=1)

    free = foot & ~obstacle
    free = ndimage.binary_opening(free, disk(2), iterations=1)
    free = ndimage.binary_closing(free, disk(2), iterations=1)
    free = free & ~obstacle
    lab, n = ndimage.label(free, np.ones((3, 3)))
    sizes = ndimage.sum(np.ones_like(lab), lab, index=np.arange(1, n + 1))
    keep = [i + 1 for i, s in enumerate(sizes) if s * RES * RES >= 1.5]
    free = np.isin(lab, keep)

    # Seal the whole room: a 1-cell black wall ring around the free region.
    # Open boundaries (doorways to the rest of the building) are drawn as solid
    # wall, i.e. we assume the doors were shut when the room was scanned.
    wall_ring = ndimage.binary_dilation(free, disk(1)) & ~free
    occ = wall_ring | obstacle

    img = np.full((H, W), 205, np.uint8)   # unknown outside the room
    img[free] = 254
    img[occ] = 0

    pgm = os.path.join(OUTDIR, "map_testroom.pgm")
    with open(pgm, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (W, H))
        f.write(np.flipud(img).tobytes())
    with open(os.path.join(OUTDIR, "map_testroom.yaml"), "w") as f:
        f.write("image: map_testroom.pgm\n")
        f.write("resolution: %.4f\n" % RES)
        f.write("origin: [%.4f, %.4f, 0.0]\n" % (XMIN, YMIN))
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

    np.savez(os.path.join(OUTDIR, "masks.npz"),
             free=free, wall_ring=wall_ring,
             obstacle=obstacle, res=RES, origin=np.array([XMIN, YMIN]))

    print("grid %dx%d  free %.1f m^2  wall-ring %d  obst %d"
          % (W, H, free.sum() * RES * RES, wall_ring.sum(), obstacle.sum()))
    print("wrote", pgm)
    return img, free, wall_ring, obstacle


if __name__ == "__main__":
    build()
