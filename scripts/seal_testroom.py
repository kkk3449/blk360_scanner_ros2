#!/usr/bin/env python3
"""Turn the *open* e57-derived testroom occupancy grid into an *enclosed indoor*
one: keep all the real scanned furniture/structure, but (1) seal the perimeter
into a closed loop so frontier exploration can't leak out into the open ground
plane, and (2) add a couple of interior partition walls (with wide doorways) so
the space reads as connected rooms instead of one big hall.

Everything downstream is regenerated from the modified grid so the world, the
map_server pgm/yaml, the preview and the spawn point stay mutually consistent:

  scripts/seal_testroom.py            # in-place: maps/testroom.* + worlds/testroom.world

Originals are backed up to *_open.* / testroom_open.world the first time.

Re-uses occ_to_world.py (wall-box merge, world emitter, free-spawn) and
e57_to_map.py (pgm/yaml/preview writers) so the encoding can't drift.
"""
import os
import sys
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import occ_to_world as ow            # noqa: E402
import e57_to_map as em             # noqa: E402

WS = os.path.dirname(HERE)
MAPS = os.path.join(WS, "src", "blk360_bringup", "maps")
WORLDS = os.path.join(WS, "src", "blk360_bringup", "worlds")
NAME = "testroom"
RES_DOOR_M = 1.20          # doorway clear width
WALL_T_CELLS = 3           # interior/partition wall thickness (~0.15 m)
WALL_HEIGHT = 2.0
ADD_PARTITIONS = False     # seal perimeter only -> one enclosed room (no rooms split)
LANDSCAPE_ASPECT = 2.2     # crop the vertical (y) extent to width/aspect -> wide
                           # rectangle. Set None to keep the full scanned height.
CROP_BIAS_BOTTOM = 0.8     # of the rows removed for the crop, fraction taken from
                           # the bottom (low y). 0.5=symmetric, ->1.0 keeps the top.
TOP_TRIM_EXTRA = 7         # extra cells trimmed off the top (high y) only, after
                           # the crop, leaving the bottom edge unchanged (~0.35 m).


def longest_free_run(mask_1d):
    """Return (lo, hi) inclusive index of the longest True run, or None."""
    best = None
    i = 0
    n = len(mask_1d)
    while i < n:
        if mask_1d[i]:
            j = i
            while j < n and mask_1d[j]:
                j += 1
            if best is None or (j - i) > (best[1] - best[0] + 1):
                best = (i, j - 1)
            i = j
        else:
            i += 1
    return best


def _largest_component(mask):
    """Size (cells) of the largest 4-connected True component in mask."""
    H, W = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    best = 0
    for sj in range(H):
        for si in range(W):
            if mask[sj, si] and not seen[sj, si]:
                q = deque([(sj, si)]); seen[sj, si] = True; s = 0
                while q:
                    j, i = q.popleft(); s += 1
                    for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        nj, ni = j + dj, i + di
                        if 0 <= nj < H and 0 <= ni < W and mask[nj, ni] and not seen[nj, ni]:
                            seen[nj, ni] = True; q.append((nj, ni))
                best = max(best, s)
    return best


def flood_free_count(occ, seed_rc):
    """4-connected free-cell count reachable from seed (BFS)."""
    H, W = occ.shape
    seen = np.zeros_like(occ, dtype=bool)
    sj, si = seed_rc
    if occ[sj, si]:
        return 0, seen
    q = deque([(sj, si)])
    seen[sj, si] = True
    cnt = 0
    while q:
        j, i = q.popleft()
        cnt += 1
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nj, ni = j + dj, i + di
            if 0 <= nj < H and 0 <= ni < W and not seen[nj, ni] and not occ[nj, ni]:
                seen[nj, ni] = True
                q.append((nj, ni))
    return cnt, seen


def main():
    meta_path = os.path.join(MAPS, NAME + "_meta.npz")
    # Always start from the pristine *open* grid if a backup exists, so this
    # script is idempotent no matter how many times it has run before.
    open_meta = os.path.join(MAPS, NAME + "_open_meta.npz")
    src_meta = open_meta if os.path.exists(open_meta) else meta_path
    print(f"[seal] base grid: {os.path.basename(src_meta)}")
    m = np.load(src_meta)
    occ = m["occ"].copy()
    res = float(m["res"]); xmin = float(m["xmin"]); ymin = float(m["ymin"])
    floor_z = float(m["floor_z"])
    H, W = occ.shape
    door = int(round(RES_DOOR_M / res))
    t = WALL_T_CELLS
    print(f"[seal] grid {W}x{H} @ {res} m, {int(occ.sum())} occupied cells (open)")

    # --- bounding box of the scanned structure = the room footprint ---
    rows = np.where(occ.any(axis=1))[0]
    cols = np.where(occ.any(axis=0))[0]
    r0, r1 = int(rows.min()), int(rows.max())
    c0, c1 = int(cols.min()), int(cols.max())
    print(f"[seal] room footprint rows[{r0},{r1}] cols[{c0},{c1}]"
          f"  ({(c1-c0)*res:.1f} x {(r1-r0)*res:.1f} m)")

    # --- optional: crop the vertical (Y) extent to a wide landscape rectangle ---
    if LANDSCAPE_ASPECT:
        width_cells = c1 - c0
        target_h = int(round(width_cells / LANDSCAPE_ASPECT))
        if target_h < (r1 - r0):
            remove = (r1 - r0) - target_h    # rows to trim; bias toward the bottom
            cut_bottom = int(round(remove * CROP_BIAS_BOTTOM))
            cut_top = remove - cut_bottom
            nr0 = r0 + cut_bottom            # low y  (image bottom)
            nr1 = r1 - cut_top - TOP_TRIM_EXTRA   # high y (image top), bottom fixed
            occ[:nr0, :] = False             # drop everything outside the new rows
            occ[nr1 + 1:, :] = False
            r0, r1 = nr0, nr1
            print(f"[seal] cropped Y to rows[{r0},{r1}] "
                  f"-> {(c1-c0)*res:.1f} x {(r1-r0)*res:.1f} m "
                  f"(aspect {(c1-c0)/(r1-r0):.2f})")

    # Snapshot of where the robot can currently roam (used to aim doorways at
    # genuinely open gaps, and to verify nothing gets walled off afterwards).
    base_free = ~occ

    # --- 1. seal the perimeter into a closed loop ---
    occ[r0:r0 + t, c0:c1 + 1] = True            # bottom (low y)
    occ[r1 - t + 1:r1 + 1, c0:c1 + 1] = True    # top (high y)
    occ[r0:r1 + 1, c0:c0 + t] = True            # left (low x)
    occ[r0:r1 + 1, c1 - t + 1:c1 + 1] = True    # right (high x)
    print("[seal] perimeter sealed")

    # --- 2. interior partitions with doorways centred on open gaps ---
    def add_vertical(pc, label):
        col_free = base_free[r0:r1 + 1, pc]
        run = longest_free_run(col_free)
        if run is None:
            print(f"[seal] {label}: no free gap, skipped")
            return
        lo = r0 + run[0]; hi = r0 + run[1]
        mid = (lo + hi) // 2
        g0 = max(r0 + t, mid - door // 2)
        g1 = min(r1 - t, mid + door // 2)
        occ[r0:r1 + 1, pc - t // 2:pc - t // 2 + t] = True
        occ[g0:g1 + 1, pc - t // 2:pc - t // 2 + t] = False
        print(f"[seal] {label} at col {pc}, doorway rows[{g0},{g1}]")

    def add_horizontal(pr, ca, cb, label):
        row_free = base_free[pr, ca:cb + 1]
        run = longest_free_run(row_free)
        if run is None:
            print(f"[seal] {label}: no free gap, skipped")
            return
        lo = ca + run[0]; hi = ca + run[1]
        mid = (lo + hi) // 2
        g0 = max(ca, mid - door // 2)
        g1 = min(cb, mid + door // 2)
        occ[pr - t // 2:pr - t // 2 + t, ca:cb + 1] = True
        occ[pr - t // 2:pr - t // 2 + t, g0:g1 + 1] = False
        print(f"[seal] {label} at row {pr}, doorway cols[{g0},{g1}]")

    if ADD_PARTITIONS:
        # vertical partition ~45% across -> left room | right region
        pc = c0 + int(round((c1 - c0) * 0.45))
        add_vertical(pc, "v-partition")
        # horizontal partition splitting the right region into two rooms
        pr = r0 + int(round((r1 - r0) * 0.52))
        add_horizontal(pr, pc, c1, "h-partition")
    else:
        print("[seal] partitions disabled -> single enclosed room")

    # --- connectivity guard: everything that used to be reachable must stay so ---
    sx, sy = ow.free_spawn(occ, res, xmin, ymin)
    si = int((sx - xmin) / res); sj = int((sy - ymin) / res)
    reach, seen = flood_free_count(occ, (sj, si))
    # free cells that existed before, inside the footprint, now unreachable
    interior = np.zeros_like(occ)
    interior[r0:r1 + 1, c0:c1 + 1] = True
    orphan = base_free & interior & (~occ) & (~seen)
    n_orphan = int(orphan.sum())
    print(f"[seal] spawn ({sx:.2f},{sy:.2f}) reaches {reach} free cells; "
          f"{n_orphan} previously-free interior cells now orphaned")
    # Largest orphaned connected component — a big one means a real room got
    # walled off (bad); small ones are just pockets behind furniture (harmless,
    # fully enclosed so they never become frontiers).
    biggest = _largest_component(orphan)
    if biggest * res * res > 1.0:
        print(f"[seal] WARNING: {biggest*res*res:.1f} m2 region walled off — "
              "check doorways/partition placement")

    print(f"[seal] occupied cells now {int(occ.sum())} (enclosed)")

    # --- regenerate world via occ_to_world's merge + emitter ---
    rects = ow.merge_rects(occ)
    parts = [ow.HEADER.format(name=NAME)]
    for k, (i0, i1, j0, j1) in enumerate(rects):
        sx_ = (i1 - i0 + 1) * res
        sy_ = (j1 - j0 + 1) * res
        cx = xmin + (i0 + (i1 - i0 + 1) / 2.0) * res
        cy = ymin + (j0 + (j1 - j0 + 1) / 2.0) * res
        parts.append(ow.BOX.format(k=k, cx=cx, cy=cy, cz=WALL_HEIGHT / 2.0,
                                   sx=sx_, sy=sy_, sz=WALL_HEIGHT))
    parts.append(ow.FOOTER)
    world_path = os.path.join(WORLDS, NAME + ".world")
    _backup(world_path, os.path.join(WORLDS, NAME + "_open.world"))
    with open(world_path, "w") as f:
        f.write("".join(parts))
    print(f"[seal] wrote {world_path}  ({len(rects)} wall boxes)")

    # --- regenerate pgm / yaml / preview (ROS map convention) ---
    img = np.full((H, W), 254, dtype=np.uint8)
    img[occ] = 0
    img_pgm = np.flipud(img)
    pgm = os.path.join(MAPS, NAME + ".pgm")
    yaml = os.path.join(MAPS, NAME + ".yaml")
    preview = os.path.join(MAPS, NAME + "_preview.png")
    for p, b in ((pgm, NAME + "_open.pgm"),
                 (yaml, NAME + "_open.yaml"),
                 (preview, NAME + "_open_preview.png"),
                 (meta_path, NAME + "_open_meta.npz")):
        _backup(p, os.path.join(MAPS, b))
    em._write_pgm(pgm, img_pgm)
    with open(yaml, "w") as f:
        f.write(f"image: {NAME}.pgm\nresolution: {res}\n")
        f.write(f"origin: [{xmin:.4f}, {ymin:.4f}, 0.0]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    try:
        em._write_png_preview(preview, img_pgm)
    except Exception as ex:
        print(f"[seal] preview skipped: {ex}")
    np.savez(meta_path, occ=occ, res=res, xmin=xmin, ymin=ymin, floor_z=floor_z)
    print(f"[seal] wrote {pgm}, {yaml}, {preview}, {meta_path}")

    # --- ground-truth map for registration vs the SLAM /map ---
    # Reachable interior = free (254), walls = occupied (0), everything else
    # (exterior + furniture-sealed pockets) = unknown (205). This mirrors what
    # Cartographer actually produces, so it overlays the live /map cleanly.
    # Same resolution + origin as testroom.yaml -> identical coordinate frame.
    gt = np.full((H, W), 205, dtype=np.uint8)
    gt[seen] = 254
    gt[occ] = 0
    gt_pgm = np.flipud(gt)
    gtp = os.path.join(MAPS, NAME + "_gt.pgm")
    gty = os.path.join(MAPS, NAME + "_gt.yaml")
    gtprev = os.path.join(MAPS, NAME + "_gt_preview.png")
    em._write_pgm(gtp, gt_pgm)
    with open(gty, "w") as f:
        f.write(f"image: {NAME}_gt.pgm\nresolution: {res}\n")
        f.write(f"origin: [{xmin:.4f}, {ymin:.4f}, 0.0]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    try:
        em._write_png_preview(gtprev, gt_pgm)
    except Exception as ex:
        print(f"[seal] gt preview skipped: {ex}")
    print(f"[seal] wrote GT {gtp} + {gty} (free/occupied/unknown)")
    print(f"[seal] SUGGESTED_SPAWN x={sx:.2f} y={sy:.2f}")
    print("[seal] SEAL_DONE")


def _backup(src, dst):
    if os.path.exists(src) and not os.path.exists(dst):
        import shutil
        shutil.copy2(src, dst)
        print(f"[seal] backup {os.path.basename(src)} -> {os.path.basename(dst)}")


if __name__ == "__main__":
    main()
