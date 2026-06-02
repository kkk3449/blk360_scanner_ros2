#!/usr/bin/env python3
"""Turn a testroom occupancy grid (from e57_to_map.py) into a Gazebo (gz Harmonic)
SDF world the TurtleBot3 can be spawned into.

Occupied cells are merged into axis-aligned wall boxes (horizontal run merge +
vertical merge of identical spans) to keep the SDF small. Reuses the same gz
system plugins as turtlebot3_world.world, with an inline sun + ground plane so
the world needs no network/Fuel access. Also computes a collision-free spawn
point near the map centre and prints it.

Usage: occ_to_world.py <meta.npz> <out.world> [--wall-height 2.0] [--name testroom]
"""
import argparse

import numpy as np

HEADER = """<?xml version="1.0"?>
<sdf version="1.8">
  <world name="{name}">
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.4 0.4 0.4 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="{name}_walls">
      <static>true</static>
      <link name="walls">
"""

FOOTER = """      </link>
    </model>
  </world>
</sdf>
"""

BOX = """        <collision name="c{k}">
          <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name="v{k}">
          <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>0.5 0.5 0.55 1</ambient><diffuse>0.6 0.6 0.65 1</diffuse></material>
        </visual>
"""


def merge_rects(occ):
    """Horizontal run merge + vertical merge of identical [i0,i1] spans.
    Returns list of (i0, i1, j0, j1) inclusive cell rectangles."""
    H, Wd = occ.shape
    open_rects = {}   # (i0,i1) -> j0  (currently extending downward)
    done = []
    for j in range(H):
        row_runs = set()
        i = 0
        row = occ[j]
        while i < Wd:
            if row[i]:
                i0 = i
                while i < Wd and row[i]:
                    i += 1
                row_runs.add((i0, i - 1))
            else:
                i += 1
        # close spans that didn't continue
        for span, j0 in list(open_rects.items()):
            if span not in row_runs:
                done.append((span[0], span[1], j0, j - 1))
                del open_rects[span]
        # open new spans
        for span in row_runs:
            if span not in open_rects:
                open_rects[span] = j
    for span, j0 in open_rects.items():
        done.append((span[0], span[1], j0, H - 1))
    return done


def free_spawn(occ, res, xmin, ymin, margin_cells=8):
    """Pick a free cell, far from walls, nearest the map centre. Returns (x,y)."""
    H, Wd = occ.shape
    # distance-to-wall via simple multi-pass (numpy BFS-ish using iterative dilation)
    free = ~occ
    dist = np.zeros((H, Wd), np.int32)
    cur = free.copy()
    for d in range(1, margin_cells + 2):
        nb = np.zeros_like(cur)
        nb[1:, :] |= cur[:-1, :]; nb[:-1, :] |= cur[1:, :]
        nb[:, 1:] |= cur[:, :-1]; nb[:, :-1] |= cur[:, 1:]
        cur = cur & nb
        dist[cur] = d
    cand = dist >= margin_cells
    if not cand.any():
        cand = dist >= max(1, dist.max() - 1)
    jj, ii = np.where(cand)
    cj, ci = H / 2.0, Wd / 2.0
    k = np.argmin((jj - cj) ** 2 + (ii - ci) ** 2)
    x = xmin + (ii[k] + 0.5) * res
    y = ymin + (jj[k] + 0.5) * res
    return float(x), float(y)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("meta")
    ap.add_argument("out")
    ap.add_argument("--wall-height", type=float, default=2.0)
    ap.add_argument("--name", default="testroom")
    args = ap.parse_args()

    m = np.load(args.meta)
    occ = m["occ"]
    res = float(m["res"]); xmin = float(m["xmin"]); ymin = float(m["ymin"])
    H, Wd = occ.shape
    print(f"[world] occ {Wd}x{H} @ {res} m, {int(occ.sum())} occupied cells")

    rects = merge_rects(occ)
    print(f"[world] merged into {len(rects)} wall boxes")

    parts = [HEADER.format(name=args.name)]
    hz = args.wall_height
    for k, (i0, i1, j0, j1) in enumerate(rects):
        sx = (i1 - i0 + 1) * res
        sy = (j1 - j0 + 1) * res
        cx = xmin + (i0 + (i1 - i0 + 1) / 2.0) * res
        cy = ymin + (j0 + (j1 - j0 + 1) / 2.0) * res
        parts.append(BOX.format(k=k, cx=cx, cy=cy, cz=hz / 2.0, sx=sx, sy=sy, sz=hz))
    parts.append(FOOTER)
    with open(args.out, "w") as f:
        f.write("".join(parts))
    print(f"[world] wrote {args.out}")

    sx, sy = free_spawn(occ, res, xmin, ymin)
    print(f"[world] SUGGESTED_SPAWN x={sx:.2f} y={sy:.2f}")
    print("[world] WORLD_DONE")


if __name__ == "__main__":
    main()
