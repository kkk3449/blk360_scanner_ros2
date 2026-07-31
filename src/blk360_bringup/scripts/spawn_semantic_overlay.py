#!/usr/bin/env python3
"""Spawn the TOSM object layer into a running Gazebo world as color-coded,
visual-only boxes (no collision — navigation and lidar are unaffected).

Each present KG object becomes a translucent static box at its map pose,
colored by semantic type (same idea as the UI overlay palette), so the
Gazebo 3D view distinguishes objects instead of showing bare walls.
Verified objects are opaque-ish; unverified clutter is faint gray.

  python3 spawn_semantic_overlay.py [--world visn2_room] [--clear]
"""
import argparse
import colorsys
import hashlib
import json
import math
import subprocess

KG = ("/home/caselab/Downloads/Cyclone360_data/blk360_seg/outputs/"
      "testroom_epochs_kg.json")
FLOOR_Z = -1.05          # KG (vis_n2) z of the floor; Gazebo floor is z=0


def type_color(t):
    h = int(hashlib.sha1(t.encode()).hexdigest(), 16) % 360 / 360.0
    r, g, b = colorsys.hsv_to_rgb(h, 0.75, 0.9)
    return r, g, b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="visn2_room")
    ap.add_argument("--clear", action="store_true",
                    help="remove previously spawned overlay models")
    args = ap.parse_args()

    g = json.load(open(KG))
    nodes = [n for n in g["nodes"] if n.get("presence") == "present"]
    if args.clear:
        for n in nodes:
            subprocess.run(
                ["gz", "service", "-s", f"/world/{args.world}/remove",
                 "--reqtype", "gz.msgs.Entity",
                 "--reptype", "gz.msgs.Boolean", "--timeout", "2000",
                 "--req", f'name: "sem_{n["name"]}" type: MODEL'],
                capture_output=True)
        print(f"cleared {len(nodes)} overlay models")
        return

    ok = 0
    for n in nodes:
        d = n["dimensions"]
        st = str(n.get("status", ""))
        if st.startswith("verified") and n["type"] != "clutter":
            r, gg, b = type_color(n["type"])
            a = 0.55
        else:
            r, gg, b, a = 0.55, 0.55, 0.55, 0.18
        x, y = n["pose"]["x"], n["pose"]["y"]
        z = n["pose"].get("z", 0.0) - FLOOR_Z
        z = max(z, d["height"] / 2 + 0.01)
        th = n["pose"].get("theta", 0.0)
        if not all(math.isfinite(v) for v in (x, y, z, th)):
            continue
        sdf = (f'<sdf version="1.9"><model name="sem_{n["name"]}">'
               f'<static>true</static>'
               f'<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {th:.3f}</pose>'
               f'<link name="l"><visual name="v"><geometry><box><size>'
               f'{max(d["length"], 0.05):.3f} {max(d["width"], 0.05):.3f} '
               f'{max(d["height"], 0.05):.3f}</size></box></geometry>'
               f'<material><ambient>{r:.2f} {gg:.2f} {b:.2f} {a}</ambient>'
               f'<diffuse>{r:.2f} {gg:.2f} {b:.2f} {a}</diffuse>'
               f'</material><transparency>{1 - a:.2f}</transparency>'
               f'</visual></link></model></sdf>')
        req = f'sdf: "{sdf.replace(chr(34), chr(92) + chr(34))}"'
        p = subprocess.run(
            ["gz", "service", "-s", f"/world/{args.world}/create",
             "--reqtype", "gz.msgs.EntityFactory",
             "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
             "--req", req], capture_output=True, text=True)
        if "true" in p.stdout:
            ok += 1
    print(f"spawned {ok}/{len(nodes)} semantic overlay boxes "
          f"into world '{args.world}'")


if __name__ == "__main__":
    main()
