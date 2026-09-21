"""Simulated VDA 5050 AGV for the Isaac digital twin (pure Python, no ROS).

Executes VDA 5050 orders in simulation: plans on the KG occupancy grid
(inflated A*), follows the path with a trapezoidal speed profile, and
publishes its own `state` / `visualization` / `connection` under a second
serialNumber (e.g. ammr20-twin). `state.information` carries the
prediction the console compares against the real robot:

    infoType "simPrediction": pathLength [m], eta [s], plannedAt [unix s]

Frames: the AGV-facing side (orders, state) is in the AGV map frame; the
planner works in the KG frame; `agv_to_kg` / `kg_to_agv` are supplied by
the caller (identity in the local test).
"""
import heapq
import math
import os
import re
import threading
import time

import numpy as np


# ---------------------------------------------------------------- map -----
def load_grid(yaml_path, cell=0.10, inflate=0.45):
    """ROS map yaml/pgm -> (occupied bool grid at `cell` m, origin (ox, oy)).
    Occupied = walls inflated by `inflate` m; unknown counts as free (the
    console map already marks unknown as dark outside the room)."""
    meta = dict(re.findall(r"(\w+):\s*(.+)", open(yaml_path).read()))
    res = float(meta["resolution"])
    ox, oy = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]
    pgm = os.path.join(os.path.dirname(yaml_path), meta["image"].strip())
    with open(pgm, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = [int(v) for v in line.split()]
        maxv = int(f.readline())
        img = np.frombuffer(f.read(), dtype=np.uint8 if maxv < 256 else ">u2",
                            count=w * h).reshape(h, w)
    occ_thr = float(meta.get("occupied_thresh", 0.65))
    negate = int(meta.get("negate", 0))
    p = (img / maxv) if negate else (1.0 - img / maxv)
    occ = p > occ_thr                        # row 0 = top of image (max y)
    occ = occ[::-1]                          # row 0 = min y
    k = max(1, int(round(cell / res)))
    H, W = occ.shape[0] // k, occ.shape[1] // k
    coarse = occ[:H * k, :W * k].reshape(H, k, W, k).any(axis=(1, 3))
    r = int(math.ceil(inflate / cell))
    if r > 0:
        ys, xs = np.nonzero(coarse)
        inflated = coarse.copy()
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        disk = (yy * yy + xx * xx) <= r * r
        for y, x in zip(ys, xs):
            y0, y1 = max(0, y - r), min(H, y + r + 1)
            x0, x1 = max(0, x - r), min(W, x + r + 1)
            inflated[y0:y1, x0:x1] |= disk[y0 - y + r:y1 - y + r,
                                           x0 - x + r:x1 - x + r]
        coarse = inflated
    return coarse, (ox, oy), cell


def astar(grid, origin, cell, start, goal, clear_radius=1.2):
    """8-connected A* on the inflated grid; returns [(x, y), ...] in metres
    (start..goal) or None. Cells within `clear_radius` of the start are
    treated as free (the AGV's own footprint is in the scan-derived map and
    Nav2 likewise clears the robot footprint); start/goal cells inside
    obstacles are snapped to the nearest free cell within 0.6 m."""
    H, W = grid.shape
    ox, oy = origin
    if clear_radius > 0:
        grid = grid.copy()
        r0, c0 = int((start[1] - oy) / cell), int((start[0] - ox) / cell)
        rr = int(math.ceil(clear_radius / cell))
        y0, y1 = max(0, r0 - rr), min(H, r0 + rr + 1)
        x0, x1 = max(0, c0 - rr), min(W, c0 + rr + 1)
        yy, xx = np.mgrid[y0:y1, x0:x1]
        grid[y0:y1, x0:x1] &= ~(((yy - r0) ** 2 + (xx - c0) ** 2) <= rr * rr)

    def to_cell(p):
        return (int((p[1] - oy) / cell), int((p[0] - ox) / cell))

    def to_xy(c):
        return (ox + (c[1] + 0.5) * cell, oy + (c[0] + 0.5) * cell)

    def free(c):
        return 0 <= c[0] < H and 0 <= c[1] < W and not grid[c[0], c[1]]

    def snap(c):
        if free(c):
            return c
        r = int(0.6 / cell) + 1
        best, bd = None, 1e9
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                n = (c[0] + dy, c[1] + dx)
                d = dy * dy + dx * dx
                if d < bd and free(n):
                    best, bd = n, d
        return best

    s, g = snap(to_cell(start)), snap(to_cell(goal))
    if s is None or g is None:
        return None
    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    h = lambda c: math.hypot(c[0] - g[0], c[1] - g[1])   # noqa: E731
    openq = [(h(s), 0.0, s)]
    came, gsc = {s: None}, {s: 0.0}
    while openq:
        _, gc, c = heapq.heappop(openq)
        if c == g:
            path = []
            while c is not None:
                path.append(to_xy(c))
                c = came[c]
            path.reverse()
            path[0], path[-1] = (start[0], start[1]), (goal[0], goal[1])
            return _simplify(path, grid, origin, cell)
        if gc > gsc.get(c, 1e18):
            continue
        for dy, dx, w in nbrs:
            n = (c[0] + dy, c[1] + dx)
            if not free(n):
                continue
            if w > 1.0 and (grid[c[0] + dy, c[1]] or grid[c[0], c[1] + dx]):
                continue                     # no corner cutting
            ng = gc + w
            if ng < gsc.get(n, 1e18):
                gsc[n], came[n] = ng, c
                heapq.heappush(openq, (ng + h(n), ng, n))
    return None


def _simplify(path, grid, origin, cell):
    """Greedy line-of-sight shortcutting (keeps the path off inflated cells)."""
    if len(path) < 3:
        return path
    ox, oy = origin

    def clear(a, b):
        n = int(math.hypot(b[0] - a[0], b[1] - a[1]) / (cell * 0.5)) + 1
        for i in range(n + 1):
            t = i / n
            x, y = a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
            r, c = int((y - oy) / cell), int((x - ox) / cell)
            if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]) or grid[r, c]:
                return False
        return True

    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not clear(path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def path_length(path):
    return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
               for i in range(len(path) - 1))


def trapezoid_time(L, vmax, acc):
    """Travel time for distance L with symmetric accel/decel."""
    d_acc = vmax * vmax / (2 * acc)
    if L < 2 * d_acc:
        return 2 * math.sqrt(L / acc)
    return 2 * vmax / acc + (L - 2 * d_acc) / vmax


# ---------------------------------------------------------------- AGV -----
class SimAgv:
    """VDA 5050 AGV endpoint driven by `step(dt)` from the sim loop."""

    def __init__(self, V, mqtt, manufacturer, serial, map_id, grid, origin,
                 cell, agv_to_kg, kg_to_agv, vmax=0.8, acc=0.5,
                 arrive=0.35, log=print):
        self.V, self.mq = V, mqtt
        self.man, self.serial, self.map_id = manufacturer, serial, map_id
        self.grid, self.origin, self.cell = grid, origin, cell
        self.a2k, self.k2a = agv_to_kg, kg_to_agv
        self.vmax, self.acc, self.arrive = vmax, acc, arrive
        self.log = log
        self.hdr = V.Header(manufacturer, serial)
        self.lock = threading.Lock()
        self.pose = None                    # (x, y, yaw) KG frame
        self.v = 0.0
        self.order_id, self.order_upd = "", 0
        self.nodes, self.edges = [], []     # remaining
        self.last_node, self.last_seq = "", 0
        self.path, self.s, self.L = None, 0.0, 0.0   # current leg
        self.target = None
        self.paused = False
        self.actions, self.errors, self.info = [], [], []
        self.t_state = 0.0
        self.t_vis = 0.0
        self.dirty = False
        self.on_path = None                 # callback(path_kg or None)
        mqtt.subscribe(V.topic(manufacturer, serial, "order"), self._on_order)
        mqtt.subscribe(V.topic(manufacturer, serial, "instantActions"),
                       self._on_instant)

    # -- MQTT in (paho thread)
    def announce(self):
        self.mq.publish(self.V.topic(self.man, self.serial, "connection"),
                        self.V.make_connection(self.hdr, "ONLINE"), retain=True)

    def _on_order(self, pt, m):
        nodes = sorted([n for n in m.get("nodes", []) if n.get("released", True)],
                       key=lambda n: n["sequenceId"])
        edges = sorted([e for e in m.get("edges", []) if e.get("released", True)],
                       key=lambda e: e["sequenceId"])
        oid, upd = m.get("orderId", ""), int(m.get("orderUpdateId", 0))
        with self.lock:
            if not nodes:
                self.errors = [self.V.error("orderError", "WARNING", "no nodes",
                                            {"orderId": oid})]
                self.dirty = True
                return
            if oid == self.order_id and upd <= self.order_upd:
                return
            self.order_id, self.order_upd = oid, upd
            self.nodes, self.edges = nodes, edges
            self.target, self.path = None, None
            self.errors, self.info = [], []
            self.paused = False
            self.dirty = True
        self.log(f"[sim-agv] order {oid}: {len(nodes)} nodes")
        self._plan_next()

    def _on_instant(self, pt, m):
        for act in m.get("actions", []):
            typ, aid = act.get("actionType"), act.get("actionId", "")
            prm = {p["key"]: p["value"] for p in act.get("actionParameters", [])}
            status, desc = "FINISHED", ""
            with self.lock:
                if typ == "cancelOrder":
                    self.nodes, self.edges, self.target, self.path = [], [], None, None
                    self.v = 0.0
                elif typ == "startPause":
                    self.paused = True
                elif typ == "stopPause":
                    self.paused = False
                elif typ == "initPosition":
                    self.pose = self.a2k(float(prm.get("x", 0)), float(prm.get("y", 0)),
                                         float(prm.get("theta", 0)))
                    self.v = 0.0
                    self.last_node = str(prm.get("lastNodeId", "") or "")
                else:
                    status, desc = "FAILED", f"unsupported {typ}"
                self.actions = (self.actions +
                                [self.V.action_state(aid, typ, status, desc)])[-10:]
                self.dirty = True
            if typ == "cancelOrder" and self.on_path:
                self.on_path(None)

    # -- planning (paho thread; heavy part off the render loop)
    def _plan_next(self):
        with self.lock:
            if self.target is not None or not self.nodes or self.pose is None:
                return
            nd = self.nodes[0]
            npz = nd["nodePosition"]
            goal = self.a2k(float(npz["x"]), float(npz["y"]), float(npz.get("theta", 0.0)))
            start = self.pose
        d0 = math.hypot(goal[0] - start[0], goal[1] - start[1])
        tol = float(npz.get("allowedDeviationXY", self.arrive)) or self.arrive
        if d0 < tol:                               # already there (node 0)
            self._reach(nd)
            return
        t0 = time.time()
        path = astar(self.grid, self.origin, self.cell, start[:2], goal[:2])
        if path is None:
            with self.lock:
                self.errors = [self.V.error("noPath", "FATAL",
                                            f"no collision-free path to {nd['nodeId']}",
                                            {"orderId": self.order_id,
                                             "nodeId": nd["nodeId"]})]
                self.nodes, self.edges, self.target = [], [], None
                self.dirty = True
            self.log(f"[sim-agv] NO PATH to {nd['nodeId']}")
            if self.on_path:
                self.on_path(None)
            return
        L = path_length(path)
        eta = trapezoid_time(L, self.vmax, self.acc)
        with self.lock:
            self.target, self.path, self.s, self.L = nd, path, 0.0, L
            self.goal_yaw = goal[2]
            self.info = [{"infoType": "simPrediction", "infoLevel": "INFO",
                          "infoDescription": f"path {L:.2f} m, eta {eta:.1f} s",
                          "infoReferences": [
                              {"referenceKey": "pathLength", "referenceValue": f"{L:.3f}"},
                              {"referenceKey": "eta", "referenceValue": f"{eta:.2f}"},
                              {"referenceKey": "nodeId", "referenceValue": nd["nodeId"]},
                              {"referenceKey": "plannedAt", "referenceValue": f"{time.time():.3f}"}]},
                         {"infoType": "simPath", "infoLevel": "INFO",
                          "infoDescription": "planned path waypoints (AGV map frame)",
                          "infoReferences": [
                              {"referenceKey": "nodeId", "referenceValue": nd["nodeId"]},
                              {"referenceKey": "points", "referenceValue": ";".join(
                                  "%.2f,%.2f" % self.k2a(x, y, 0.0)[:2] for x, y in path)}]}]
            self.dirty = True
        self.log(f"[sim-agv] planned {nd['nodeId']}: {len(path)} pts, {L:.2f} m, "
                 f"eta {eta:.1f} s ({(time.time()-t0)*1000:.0f} ms)")
        if self.on_path:
            self.on_path(path)

    def _reach(self, nd):
        with self.lock:
            self.last_node, self.last_seq = nd["nodeId"], nd["sequenceId"]
            if self.nodes and self.nodes[0]["nodeId"] == nd["nodeId"]:
                self.nodes.pop(0)
            self.edges = [e for e in self.edges if e["sequenceId"] > nd["sequenceId"]]
            self.target, self.path = None, None
            self.v = 0.0
            self.dirty = True
            more = bool(self.nodes)
        self.log(f"[sim-agv] node {nd['nodeId']} reached; {len(self.nodes)} left")
        if more:
            self._plan_next()
        elif self.on_path:
            self.on_path(None)

    # -- motion (sim loop)
    def step(self, dt, now):
        with self.lock:
            tgt, path, paused = self.target, self.path, self.paused
        if tgt is not None and path is not None and not paused and dt > 0:
            with self.lock:
                remaining = self.L - self.s
                v_stop = math.sqrt(max(0.0, 2 * self.acc * remaining))
                self.v = min(self.vmax, self.v + self.acc * dt, v_stop)
                self.s = min(self.L, self.s + self.v * dt)
                x, y, yaw = self._along(path, self.s)
                self.pose = (x, y, yaw)
                done = self.L - self.s < 1e-3
            if done:
                with self.lock:
                    self.pose = (path[-1][0], path[-1][1], self.goal_yaw)
                self._reach(tgt)
        # publishing
        if now - self.t_vis > 0.05:
            self.t_vis = now
            self._pub_vis()
        if self.dirty or now - self.t_state > 1.0:
            self.dirty = False
            self.t_state = now
            self._pub_state()

    @staticmethod
    def _along(path, s):
        acc = 0.0
        for i in range(len(path) - 1):
            a, b = path[i], path[i + 1]
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            if acc + seg >= s or i == len(path) - 2:
                t = 0.0 if seg == 0 else min(1.0, (s - acc) / seg)
                return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]),
                        math.atan2(b[1] - a[1], b[0] - a[0]))
            acc += seg
        return (path[-1][0], path[-1][1], 0.0)

    # -- MQTT out
    def _agv_pos(self):
        if self.pose is None:
            return {"x": 0.0, "y": 0.0, "theta": 0.0, "mapId": self.map_id,
                    "positionInitialized": False}
        x, y, th = self.k2a(*self.pose)
        return {"x": x, "y": y, "theta": th, "mapId": self.map_id,
                "positionInitialized": True}

    def _pub_vis(self):
        if not self.mq.connected.is_set():
            return
        with self.lock:
            pos, v, yaw = self._agv_pos(), self.v, (self.pose[2] if self.pose else 0.0)
        self.mq.publish(self.V.topic(self.man, self.serial, "visualization"),
                        self.V.make_visualization(self.hdr, pos["x"], pos["y"],
                                                  pos["theta"], self.map_id,
                                                  vx=v * math.cos(yaw), vy=v * math.sin(yaw)),
                        qos=0)

    def _pub_state(self):
        if not self.mq.connected.is_set():
            return
        with self.lock:
            m = self.V.make_state(
                self.hdr, order_id=self.order_id, order_update_id=self.order_upd,
                last_node_id=self.last_node, last_node_seq=self.last_seq,
                node_states=[{"nodeId": n["nodeId"], "sequenceId": n["sequenceId"],
                              "released": True, "nodePosition": n["nodePosition"]}
                             for n in self.nodes],
                edge_states=[{"edgeId": e["edgeId"], "sequenceId": e["sequenceId"],
                              "released": True} for e in self.edges],
                agv_position=self._agv_pos(),
                velocity={"vx": self.v, "vy": 0.0, "omega": 0.0},
                driving=self.target is not None and not self.paused,
                paused=self.paused, action_states=list(self.actions),
                battery_charge=100.0, errors=list(self.errors),
                information=list(self.info))
        self.mq.publish(self.V.topic(self.man, self.serial, "state"), m)

    def snapshot(self):
        with self.lock:
            return self.pose, self.v, self.target is not None
