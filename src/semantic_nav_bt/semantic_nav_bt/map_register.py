"""Register a robot's AMCL occupancy map onto the KG (scan-derived) occupancy
map and return the SE(2) offset the real-robot bridge uses:

    p_kg = R(yaw) @ p_robot + t          (--map-offset X Y YAW_DEG)

Method: occupied cells of both maps as 2-D point sets; coarse search over yaw
with FFT cross-correlation for the translation (on a common grid), candidates
scored by the chamfer distance against the KG map's distance transform; the
best candidate is refined by point-to-nearest ICP (Kabsch) using the distance
transform's nearest-cell labels. Pure numpy/scipy/cv2 (all in the ROS python).

    python3 -m semantic_nav_bt.map_register robot.yaml kg.yaml [--init X Y YAW]
"""
import json
import math
import os
import sys

import numpy as np
import yaml


def load_map(yaml_path):
    """-> dict(occ: bool HxW (row 0 = top), res, origin (ox, oy), w, h)."""
    from PIL import Image
    y = yaml.safe_load(open(yaml_path))
    img = np.asarray(Image.open(os.path.join(os.path.dirname(yaml_path),
                                              y["image"])).convert("L"))
    occ_th = float(y.get("occupied_thresh", 0.65))
    p = (255 - img) / 255.0 if not y.get("negate", 0) else img / 255.0
    occ = p >= occ_th
    return {"occ": occ, "res": float(y["resolution"]),
            "origin": (float(y["origin"][0]), float(y["origin"][1])),
            "w": img.shape[1], "h": img.shape[0]}


def occupied_points(m):
    rows, cols = np.nonzero(m["occ"])
    x = m["origin"][0] + (cols + 0.5) * m["res"]
    y = m["origin"][1] + (m["h"] - 1 - rows + 0.5) * m["res"]
    return np.column_stack([x, y]).astype(np.float64)


def _rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


class KGField:
    """Distance transform of the KG map with nearest-cell labels."""

    def __init__(self, m, pad_m=4.0):
        import cv2
        pad = int(pad_m / m["res"])
        occ = np.pad(m["occ"], pad, constant_values=False)
        free = (~occ).astype(np.uint8)
        dist, labels = cv2.distanceTransformWithLabels(
            free, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
        self.dist = dist * m["res"]
        # label -> (row, col) of the occupied pixel
        occ_rc = np.column_stack(np.nonzero(occ))
        lab_of_occ = labels[occ]                      # label id per occ pixel
        self.lab2rc = np.zeros((int(labels.max()) + 1, 2), dtype=np.int64)
        self.lab2rc[lab_of_occ] = occ_rc
        self.labels = labels
        self.m, self.pad = m, pad
        self.H, self.W = occ.shape

    def rc(self, pts):
        """world points -> (row, col) in the padded grid (float)."""
        m = self.m
        col = (pts[:, 0] - m["origin"][0]) / m["res"] - 0.5 + self.pad
        row = (m["h"] - 1 - (pts[:, 1] - m["origin"][1]) / m["res"] + 0.5) + self.pad
        return row, col

    def query(self, pts):
        row, col = self.rc(pts)
        r = np.clip(np.rint(row).astype(int), 0, self.H - 1)
        c = np.clip(np.rint(col).astype(int), 0, self.W - 1)
        inside = (row >= 0) & (row < self.H) & (col >= 0) & (col < self.W)
        d = np.where(inside, self.dist[r, c], np.inf)
        nrc = self.lab2rc[self.labels[r, c]]
        m = self.m
        nx = m["origin"][0] + (nrc[:, 1] - self.pad + 0.5) * m["res"]
        ny = m["origin"][1] + (m["h"] - 1 - (nrc[:, 0] - self.pad) + 0.5) * m["res"]
        return d, np.column_stack([nx, ny])


def chamfer(field, pts, trim=0.5):
    d, _ = field.query(pts)
    d = np.sort(d[np.isfinite(d)])
    if len(d) == 0:
        return np.inf
    return float(d[: max(1, int(len(d) * (1 - trim)))].mean())  # trimmed mean


def coarse_search(kg, rob_pts, yaws, res):
    """FFT cross-correlation per yaw on a common grid; returns candidates."""
    kg_pts = occupied_points(kg)
    allp = np.vstack([kg_pts, rob_pts])
    lo = allp.min(0) - 2.0
    span = allp.max(0) - lo + 2.0 + rob_pts.ptp(0)
    W = int(np.ceil(span[0] / res)); H = int(np.ceil(span[1] / res))
    W2, H2 = 1 << int(np.ceil(np.log2(W))), 1 << int(np.ceil(np.log2(H)))

    def raster(p):
        g = np.zeros((H2, W2), np.float32)
        c = ((p[:, 0] - lo[0]) / res).astype(int); r = ((p[:, 1] - lo[1]) / res).astype(int)
        ok = (c >= 0) & (c < W2) & (r >= 0) & (r < H2)
        g[r[ok], c[ok]] = 1.0
        return g

    import cv2
    A = raster(kg_pts)
    A = cv2.GaussianBlur(A, (0, 0), 1.0)          # tolerance ~1 cell
    FA = np.fft.rfft2(A)
    c0 = rob_pts.mean(0)
    cands = []
    for yaw in yaws:
        q = (rob_pts - c0) @ _rot(yaw).T + c0
        B = raster(q)
        corr = np.fft.irfft2(FA * np.conj(np.fft.rfft2(B)), s=A.shape)
        idx = np.argpartition(corr.ravel(), -3)[-3:]
        for k in idx:
            r, c = divmod(int(k), W2)
            dy = (r if r < H2 // 2 else r - H2) * res
            dx = (c if c < W2 // 2 else c - W2) * res
            cands.append((float(corr.ravel()[k]), yaw, dx, dy))
    return cands, c0


def icp(field, rob_pts, yaw, t, iters=40, max_d=0.6):
    R = _rot(yaw); t = np.array(t, float)
    for _ in range(iters):
        q = rob_pts @ R.T + t
        d, nn = field.query(q)
        ok = np.isfinite(d) & (d < max_d)
        if ok.sum() < 20:
            break
        P, Q = rob_pts[ok], nn[ok]
        pc, qc = P.mean(0), Q.mean(0)
        Hm = (P - pc).T @ (Q - qc)
        U, _, Vt = np.linalg.svd(Hm)
        D = np.diag([1, np.sign(np.linalg.det(Vt.T @ U))])
        Rn = Vt.T @ D @ U.T
        tn = qc - Rn @ pc
        if np.allclose(Rn, R, atol=1e-7) and np.allclose(tn, t, atol=1e-5):
            break
        R, t = Rn, tn
    q = rob_pts @ R.T + t
    d, _ = field.query(q)
    fin = d[np.isfinite(d)]
    stats = {"mean_m": float(fin.mean()), "median_m": float(np.median(fin)),
             "inlier_frac_15cm": float((fin < 0.15).mean()),
             "inlier_frac_30cm": float((fin < 0.30).mean()), "n": int(len(fin))}
    return math.atan2(R[1, 0], R[0, 0]), t, stats


def register(robot_yaml, kg_yaml, init=None, yaw_window_deg=None, log=print):
    kg, rob = load_map(kg_yaml), load_map(robot_yaml)
    rob_pts = occupied_points(rob)
    if len(rob_pts) > 6000:                      # subsample for speed
        rob_pts = rob_pts[np.random.default_rng(0).choice(len(rob_pts), 6000, replace=False)]
    field = KGField(kg)
    res = max(kg["res"], rob["res"]) * 2         # coarse grid
    if init is not None and yaw_window_deg:
        y0 = math.radians(init[2])
        yaws = y0 + np.radians(np.arange(-yaw_window_deg, yaw_window_deg + 1, 1.0))
    else:
        yaws = np.radians(np.arange(-180, 180, 1.0))
    log(f"[reg] KG {kg['w']}x{kg['h']}@{kg['res']}  robot {rob['w']}x{rob['h']}@{rob['res']}; "
        f"{len(rob_pts)} robot occ pts; {len(yaws)} yaw hypotheses")
    cands, c0 = coarse_search(kg, rob_pts, yaws, res)
    cands.sort(key=lambda x: -x[0])
    best = None
    for score, yaw, dx, dy in cands[:40]:
        # translation from correlation: q = R(p - c0) + c0 + (dx,dy)  ->  t = c0 - R c0 + (dx,dy)
        t = c0 - _rot(yaw) @ c0 + np.array([dx, dy])
        ch = chamfer(field, rob_pts @ _rot(yaw).T + t)
        if best is None or ch < best[0]:
            best = (ch, yaw, t)
    log(f"[reg] coarse best: yaw {math.degrees(best[1]):.1f} deg, t ({best[2][0]:.2f},{best[2][1]:.2f}), "
        f"trimmed chamfer {best[0]:.3f} m")
    yaw, t, stats = icp(field, rob_pts, best[1], best[2])
    out = {"map_offset": [round(float(t[0]), 4), round(float(t[1]), 4),
                          round(math.degrees(yaw), 3)],
           "stats": stats, "robot_yaml": os.path.abspath(robot_yaml),
           "kg_yaml": os.path.abspath(kg_yaml)}
    log(f"[reg] refined: X {t[0]:.4f} Y {t[1]:.4f} YAW {math.degrees(yaw):.3f} deg; "
        f"mean {stats['mean_m']:.3f} m, inliers<15cm {stats['inlier_frac_15cm']:.0%}")
    return out


def transformed_cells(robot_yaml, offset, step=2):
    """Robot map occupied cells in the KG frame (for the console overlay)."""
    rob = load_map(robot_yaml)
    p = occupied_points(rob)[::step]
    R = _rot(math.radians(offset[2]))
    q = p @ R.T + np.array(offset[:2])
    return np.round(q, 3).tolist()


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("robot_yaml"); ap.add_argument("kg_yaml")
    ap.add_argument("--init", nargs=3, type=float, default=None, metavar=("X", "Y", "YAW_DEG"))
    ap.add_argument("--yaw-window", type=float, default=30.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    r = register(a.robot_yaml, a.kg_yaml, a.init, a.yaw_window if a.init else None)
    print(json.dumps(r, indent=1))
    if a.out:
        json.dump(r, open(a.out, "w"), indent=1)
