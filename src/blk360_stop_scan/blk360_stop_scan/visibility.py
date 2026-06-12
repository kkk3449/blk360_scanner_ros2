"""Occlusion-aware sensor coverage: ray-cast visibility regions on an occupancy grid.

The isotropic-disk coverage model treats every cell within radius R of a scan
pose as covered, even cells behind a wall. This module replaces the disk with
the visibility region

    B(s, R) = { p : |p - s| <= R  and  the segment s->p crosses no occupied cell }

computed by marching K rays outward from s on the occupancy grid (a discrete
visibility polygon). Pure numpy, no ROS imports, so it is unit-testable offline
against a saved map PGM.

Grid convention matches nav_msgs/OccupancyGrid: int values, -1 unknown,
0..100 occupancy probability, row 0 at origin (origin = world position of cell
(0,0)'s corner), row-major (H, W) with x along columns and y along rows.
"""
import numpy as np


def visible_mask(grid, resolution, origin_xy, x, y, radius,
                 num_rays=720, occ_thresh=65, unknown_blocks=True):
    """Ray-cast the visibility region B((x,y), radius) on an occupancy grid.

    Returns (mask, endpoints):
      mask      -- bool (H, W); True for cells visible (line-of-sight, known
                   free) from (x, y) within `radius`.
      endpoints -- float (num_rays, 2); world-frame end point of each ray
                   (obstacle hit or range limit), i.e. the visibility polygon
                   vertices in angular order.
    """
    H, W = grid.shape
    step = resolution * 0.5
    n_steps = max(int(radius / step), 1)
    angles = np.linspace(0.0, 2.0 * np.pi, num_rays, endpoint=False)
    radii = (np.arange(n_steps) + 1.0) * step                  # (S,)
    px = x + np.cos(angles)[:, None] * radii[None, :]          # (K, S)
    py = y + np.sin(angles)[:, None] * radii[None, :]
    col = np.floor((px - origin_xy[0]) / resolution).astype(np.int32)
    row = np.floor((py - origin_xy[1]) / resolution).astype(np.int32)
    inb = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    val = np.full(px.shape, -1, dtype=np.int16)
    val[inb] = grid[row[inb], col[inb]]
    blocked = ~inb | (val >= occ_thresh)
    if unknown_blocks:
        blocked |= val < 0
    # First blocked sample per ray; rays with no hit run to the range limit.
    any_hit = blocked.any(axis=1)
    first = np.where(any_hit, blocked.argmax(axis=1), n_steps)  # (K,)
    visible = np.arange(n_steps)[None, :] < first[:, None]      # (K, S)
    mask = np.zeros((H, W), dtype=bool)
    sel = visible & inb
    mask[row[sel], col[sel]] = True
    # Center cell is trivially visible.
    c0 = int(np.floor((x - origin_xy[0]) / resolution))
    r0 = int(np.floor((y - origin_xy[1]) / resolution))
    if 0 <= c0 < W and 0 <= r0 < H:
        mask[r0, c0] = True
    r_end = np.minimum(first, n_steps) * step
    endpoints = np.stack([x + np.cos(angles) * r_end,
                          y + np.sin(angles) * r_end], axis=1)
    return mask, endpoints


def union_visible_mask(grid, resolution, origin_xy, positions, radius, **kw):
    """Union coverage C = ∪_i B(s_i, R) over scan positions on one grid."""
    H, W = grid.shape
    cov = np.zeros((H, W), dtype=bool)
    for (sx, sy) in positions:
        m, _ = visible_mask(grid, resolution, origin_xy, sx, sy, radius, **kw)
        cov |= m
    return cov


def new_visible_ratio(grid, resolution, origin_xy, candidate_xy, positions,
                      radius, **kw):
    """Marginal-gain skip criterion for a scan candidate.

    Returns (gain, cand_area_m2, new_area_m2) where
      gain = |B(c,R) \\ C| / |B(c,R)|,  C = ∪_i B(s_i,R)
    or (None, 0, 0) when the candidate's visibility region is degenerate
    (e.g. the pose projects into an obstacle or an unmapped pocket).
    """
    cand, _ = visible_mask(grid, resolution, origin_xy,
                           candidate_xy[0], candidate_xy[1], radius, **kw)
    cell_area = resolution * resolution
    cand_cells = int(cand.sum())
    if cand_cells * cell_area < 0.5:   # < 0.5 m^2 visible: degenerate
        return None, cand_cells * cell_area, 0.0
    if positions:
        cov = union_visible_mask(grid, resolution, origin_xy, positions,
                                 radius, **kw)
        new_cells = int((cand & ~cov).sum())
    else:
        new_cells = cand_cells
    return (new_cells / cand_cells, cand_cells * cell_area,
            new_cells * cell_area)
