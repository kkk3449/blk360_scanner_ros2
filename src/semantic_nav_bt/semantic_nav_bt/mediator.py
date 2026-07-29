"""TOSM semantic mediator: resolve semantic commands into metric nav goals.

The mediator is the deliberative half of the architecture: it answers
"where is X?" from the TOSM knowledge graph (objects / places / robots),
leaving "how do I get there / what if the world changes" to the reactive
behavior tree. Only verified-tier objects are eligible targets (the
confidence gate propagates into task planning); an ungated mode exists for
the gated-vs-ungated mission-waste experiment.

Commands (JSON on /semantic_command):
  {"cmd": "goto_place",  "target": "display_briefing_area" | "region_006"}
  {"cmd": "goto_object", "target": "tv" | "tv_006"}
  {"cmd": "patrol",      "targets": ["seating_area", ...]}   # default: all
  {"cmd": "return_home"}
  {"cmd": "pick",        "target": "chair_011"}
"""
import json
import math
import os
import re

import numpy as np


class Mediator:
    def __init__(self, kg_path, places_path, naming_path=None, map_yaml=None,
                 gated=True, naming_key="T3_slic_rev4"):
        self.kg_path = kg_path
        self._kg_mtime = os.path.getmtime(kg_path)
        self.kg = json.load(open(kg_path))
        d = json.load(open(places_path))
        self.places = d["semanticPlaces"]
        self.cell_m = d["cell_m"]
        self.gated = gated
        self.names = {}
        if naming_path and os.path.exists(naming_path):
            book = json.load(open(naming_path))
            ep = book.get(naming_key) or {}
            self.names = {r: v["name"] for r, v in ep.items()
                          if isinstance(v, dict) and "name" in v}
        self._grid = None
        if map_yaml and os.path.exists(map_yaml):
            self._load_grid(map_yaml)

    # ---------------------------------------------------------------- grid --
    def _load_grid(self, yaml_path):
        meta = dict(re.findall(r"(\w+):\s*(.+)", open(yaml_path).read()))
        res = float(meta["resolution"])
        ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
        pgm = os.path.join(os.path.dirname(yaml_path), meta["image"])
        with open(pgm, "rb") as f:
            assert f.readline().strip() == b"P5"
            line = f.readline()
            while line.startswith(b"#"):
                line = f.readline()
            w, h = map(int, line.split())
            f.readline()
            img = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)
        self._grid = (np.flipud(img) > 200, res, ox, oy)   # free mask

    def _is_free(self, x, y, radius=0.25):
        if self._grid is None:
            return True
        free, res, ox, oy = self._grid
        r = max(1, int(radius / res))
        ci, cj = int((x - ox) / res), int((y - oy) / res)
        if not (r <= ci < free.shape[1] - r and r <= cj < free.shape[0] - r):
            return False
        return bool(free[cj - r:cj + r + 1, ci - r:ci + r + 1].all())

    def _nearest_free(self, x, y, radius=0.25, rmax=1.5):
        if self._is_free(x, y, radius):
            return x, y
        for rr in np.arange(0.15, rmax, 0.15):
            for a in np.linspace(0, 2 * math.pi, 16, endpoint=False):
                nx, ny = x + rr * math.cos(a), y + rr * math.sin(a)
                if self._is_free(nx, ny, radius):
                    return nx, ny
        return x, y

    # ------------------------------------------------------------- lookups --
    def objects(self, include_unverified=False):
        out = []
        for n in self.kg["nodes"]:
            if n.get("presence") == "absent":
                continue
            st = str(n.get("status", ""))
            ok = st.startswith("verified")
            if ok or include_unverified or not self.gated:
                out.append(n)
        return out

    def find_object(self, target):
        cand = self.objects(include_unverified=not self.gated)
        # exact instance name first, then type match (highest confidence)
        for n in cand:
            if n["name"] == target:
                return n
        typed = [n for n in cand if n.get("type") == target
                 or target in str(n.get("type", ""))]
        if typed:
            return max(typed, key=lambda n: n.get("confidence", 0))
        return None

    def find_place(self, target):
        for p in self.places:
            nm = self.names.get(p["name"], p["name"])
            if target in (p["name"], nm):
                return p, nm
        return None, None

    def robot(self):
        rs = self.kg.get("robots", [])
        return rs[0] if rs else None

    # ------------------------------------------------------------- resolve --
    def _approach_pose(self, n, standoff=None):
        """Free-space pose near the object, heading facing it."""
        x, y = n["pose"]["x"], n["pose"]["y"]
        d = n.get("dimensions", {})
        half = 0.5 * math.hypot(d.get("length", 0.5), d.get("width", 0.5))
        rob = self.robot()
        rad = (rob or {}).get("explicit", {}).get("limits", {}) \
            .get("nav_radius_m", 0.25)
        standoff = standoff if standoff is not None else half + rad + 0.35
        best = None
        for a in np.linspace(0, 2 * math.pi, 24, endpoint=False):
            gx, gy = x + standoff * math.cos(a), y + standoff * math.sin(a)
            if self._is_free(gx, gy, rad):
                # prefer the direction closest to the robot's current pose
                score = 0.0
                if rob:
                    rp = rob["explicit"]["pose"]
                    score = -math.hypot(gx - rp["x"], gy - rp["y"])
                if best is None or score > best[0]:
                    best = (score, gx, gy)
        if best is None:
            gx, gy = self._nearest_free(x, y, rad)
        else:
            _, gx, gy = best
        yaw = math.atan2(y - gy, x - gx)
        return {"x": round(gx, 3), "y": round(gy, 3), "yaw": round(yaw, 3)}

    def _maybe_reload(self):
        """Pick up KG edits made by the management UI without a restart."""
        try:
            mt = os.path.getmtime(self.kg_path)
            if mt != self._kg_mtime:
                self.kg = json.load(open(self.kg_path))
                self._kg_mtime = mt
        except (OSError, json.JSONDecodeError):
            pass

    def resolve(self, cmd):
        """command dict -> {"goals": [goal...]} or {"error": ...}.
        goal = {x, y, yaw, label, kind, [pick_target]}"""
        self._maybe_reload()
        c = cmd.get("cmd")
        if c == "goto_place":
            p, nm = self.find_place(cmd.get("target", ""))
            if not p:
                return {"error": f"unknown place '{cmd.get('target')}'"}
            cx, cy = p["centroid"]
            gx, gy = self._nearest_free(cx, cy)
            return {"goals": [{"x": round(gx, 3), "y": round(gy, 3),
                               "yaw": 0.0, "label": nm, "kind": "place"}]}
        if c in ("goto_object", "pick"):
            n = self.find_object(cmd.get("target", ""))
            if not n:
                tier = "verified" if self.gated else "any"
                return {"error": f"no {tier} object '{cmd.get('target')}'"}
            g = self._approach_pose(n)
            g["label"] = n["name"]
            g["kind"] = "object"
            if c == "pick":
                g["kind"] = "pick"
                g["pick_target"] = {"name": n["name"], "type": n["type"],
                                    "pose": n["pose"],
                                    "dimensions": n.get("dimensions"),
                                    "isMovable": n.get("implicit", {})
                                    .get("isMovable")}
            return {"goals": [g]}
        if c == "patrol":
            targets = cmd.get("targets") or [p["name"] for p in self.places]
            goals = []
            for t in targets:
                r = self.resolve({"cmd": "goto_place", "target": t})
                if "goals" in r:
                    goals += r["goals"]
            if not goals:
                return {"error": "no resolvable patrol targets"}
            for g in goals:
                g["kind"] = "patrol"
            return {"goals": goals}
        if c == "return_home":
            rob = self.robot()
            hp = (rob or {}).get("explicit", {}).get("pose",
                                                     {"x": 0, "y": 0})
            g = {"x": hp["x"], "y": hp["y"], "yaw": 0.0, "label": "home"}
            g["kind"] = "return_home"
            return {"goals": [g]}
        return {"error": f"unknown cmd '{c}'"}
