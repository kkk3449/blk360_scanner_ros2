#!/usr/bin/env python3
"""Digital-twin quantitative-eval report (paper table + figure).

Input: a session dir from twin_eval_logger.py plus a hand-filled
ground_truth.yaml in it. On first run without the yaml, a template listing
every checkpoint pair is generated — fill in the tape/laser-measured values.

    ground_truth.yaml
    -----------------
    distances:          # 실측 거리 [m] between checkpoint labels
      - [A, B, 3.42]
      - [B, C, 2.15]
    goals_external:     # optional: goal_id, 실측 정지-위치 오차 [m]
      - [1, 0.12]

Output: eval_report.csv, eval_fig.png (real-vs-twin scatter + per-pair error
bars) and a console table. Usage:

    python3 scripts/twin_eval_report.py [SESSION_DIR]   # default: latest
"""
import argparse
import csv
import math
import sys
from itertools import combinations
from pathlib import Path

import yaml


def load_checkpoints(session):
    cps = {}
    with open(session / "checkpoints.csv", newline="") as f:
        for row in csv.DictReader(f):
            cps[row["label"]] = row  # last fix wins if a label was redone
    return cps


def load_goal_arrivals(session):
    arrivals = []
    p = session / "goals.csv"
    if not p.exists():
        return arrivals
    with open(p, newline="") as f:
        for row in csv.DictReader(f):
            if row["event"] == "arrival" and row["err_to_goal_m"]:
                arrivals.append((int(row["goal_id"]), float(row["err_to_goal_m"])))
    return arrivals


def write_template(session, cps):
    path = session / "ground_truth.yaml"
    lines = ["# 실측값을 채우세요 (m 단위). 안 잰 쌍은 줄을 지우면 됩니다.",
             "distances:"]
    for a, b in combinations(sorted(cps), 2):
        lines.append(f"  - [{a}, {b}, ]")
    lines += ["", "# 선택: goal_id별 실측 정지-위치 오차 [m]", "goals_external:", "#  - [1, ]"]
    path.write_text("\n".join(lines) + "\n")
    return path


def dist(cp1, cp2, prefix):
    return math.hypot(float(cp1[prefix + "_x"]) - float(cp2[prefix + "_x"]),
                      float(cp1[prefix + "_y"]) - float(cp2[prefix + "_y"]))


def stats(errs):
    if not errs:
        return {}
    ae = [abs(e) for e in errs]
    return {"n": len(errs),
            "mean_abs": sum(ae) / len(ae),
            "rmse": math.sqrt(sum(e * e for e in errs) / len(errs)),
            "max_abs": max(ae),
            "mean_signed": sum(errs) / len(errs)}


def make_figure(pairs, out_png):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib 없음 — 그림 생략 (표/CSV는 생성됨)")
        return
    BLUE, GRAY, INK = "#2f6fb3", "#9aa2ad", "#1f2937"
    labels = [f"{a}–{b}" for a, b, *_ in pairs]
    real = [p[2] for p in pairs]
    twin = [p[3] for p in pairs]
    err_cm = [(t - r) * 100 for r, t in zip(real, twin)]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8.0, 3.4), dpi=150)
    for ax in (ax1, ax2):
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        ax.tick_params(colors=INK, labelsize=8)
        ax.grid(True, color="#e5e7eb", linewidth=0.6, zorder=0)
        ax.set_axisbelow(True)

    lo = min(real + twin) * 0.95
    hi = max(real + twin) * 1.05
    ax1.plot([lo, hi], [lo, hi], "--", color=GRAY, linewidth=1.0, zorder=1)
    ax1.scatter(real, twin, s=28, color=BLUE, zorder=2)
    for x, y, lb in zip(real, twin, labels):
        ax1.annotate(lb, (x, y), textcoords="offset points", xytext=(4, 4),
                     fontsize=7, color=INK)
    ax1.set_xlabel("Measured distance [m]", fontsize=9, color=INK)
    ax1.set_ylabel("Digital-twin distance [m]", fontsize=9, color=INK)
    ax1.set_title("(a) Real vs twin distance", fontsize=9, color=INK)

    xs = range(len(pairs))
    ax2.bar(xs, err_cm, width=0.6, color=BLUE, zorder=2)
    ax2.axhline(0, color=GRAY, linewidth=1.0, zorder=1)
    for x, e in zip(xs, err_cm):
        ax2.annotate(f"{e:+.1f}", (x, e), ha="center", fontsize=7, color=INK,
                     xytext=(0, 3 if e >= 0 else -10), textcoords="offset points")
    ax2.set_xticks(list(xs), labels, fontsize=8)
    ax2.set_ylabel("Twin − real [cm]", fontsize=9, color=INK)
    ax2.set_title("(b) Per-pair distance error", fontsize=9, color=INK)

    fig.tight_layout()
    fig.savefig(out_png, bbox_inches="tight")
    print(f"그림 저장: {out_png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", nargs="?", help="세션 디렉토리 (기본: 최신)")
    args = ap.parse_args()
    base = Path.home() / "ammr_twin" / "eval"
    if args.session:
        session = Path(args.session)
    else:
        sessions = sorted(base.glob("session_*"))
        if not sessions:
            sys.exit(f"세션 없음: {base}")
        session = sessions[-1]
    print(f"세션: {session}")

    cps = load_checkpoints(session)
    gt_path = session / "ground_truth.yaml"
    if not gt_path.exists():
        p = write_template(session, cps)
        sys.exit(f"실측값 템플릿 생성: {p}\n값을 채운 뒤 다시 실행하세요.")
    gt = yaml.safe_load(gt_path.read_text()) or {}

    pairs = []  # (a, b, real, twin, robot)
    for entry in gt.get("distances") or []:
        a, b, d_real = entry[0], entry[1], entry[2]
        if d_real is None:
            continue
        a, b = str(a), str(b)
        if a not in cps or b not in cps:
            print(f"경고: 체크포인트 {a} 또는 {b} 기록 없음 — 건너뜀")
            continue
        pairs.append((a, b, float(d_real),
                      dist(cps[a], cps[b], "twin"),
                      dist(cps[a], cps[b], "robot")))
    if not pairs:
        sys.exit("distances에 채워진 실측값이 없습니다.")

    twin_errs = [t - r for _, _, r, t, _ in pairs]
    robot_errs = [ro - r for _, _, r, _, ro in pairs]

    print(f"\n{'pair':8} {'real[m]':>8} {'twin[m]':>8} {'err[cm]':>8}   (robot-odom err[cm])")
    for (a, b, r, t, ro), te, re_ in zip(pairs, twin_errs, robot_errs):
        print(f"{a+'-'+b:8} {r:8.3f} {t:8.3f} {te*100:8.1f}   ({re_*100:.1f})")
    st = stats(twin_errs)
    print(f"\n트윈 거리 오차: n={st['n']}  mean|e|={st['mean_abs']*100:.1f} cm  "
          f"RMSE={st['rmse']*100:.1f} cm  max={st['max_abs']*100:.1f} cm  "
          f"bias={st['mean_signed']*100:+.1f} cm")

    arrivals = load_goal_arrivals(session)
    if arrivals:
        st_g = stats([e for _, e in arrivals])
        print(f"goal 내부 오차(트윈 좌표): n={st_g['n']}  mean={st_g['mean_abs']*100:.1f} cm  "
              f"max={st_g['max_abs']*100:.1f} cm")
    ext = [(int(g), float(e)) for g, e in (gt.get("goals_external") or []) if e is not None]
    if ext:
        st_e = stats([e for _, e in ext])
        print(f"goal 실측 오차: n={st_e['n']}  mean={st_e['mean_abs']*100:.1f} cm  "
              f"max={st_e['max_abs']*100:.1f} cm")

    out_csv = session / "eval_report.csv"
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pair", "real_m", "twin_m", "twin_err_m", "robot_err_m"])
        for (a, b, r, t, ro), te, re_ in zip(pairs, twin_errs, robot_errs):
            w.writerow([f"{a}-{b}", f"{r:.4f}", f"{t:.4f}", f"{te:.4f}", f"{re_:.4f}"])
        w.writerow([])
        w.writerow(["stat", "n", "mean_abs_m", "rmse_m", "max_abs_m", "mean_signed_m"])
        w.writerow(["twin_distance", st["n"], f"{st['mean_abs']:.4f}",
                    f"{st['rmse']:.4f}", f"{st['max_abs']:.4f}", f"{st['mean_signed']:.4f}"])
        if arrivals:
            w.writerow(["goal_internal", st_g["n"], f"{st_g['mean_abs']:.4f}",
                        f"{st_g['rmse']:.4f}", f"{st_g['max_abs']:.4f}", ""])
        if ext:
            w.writerow(["goal_external", st_e["n"], f"{st_e['mean_abs']:.4f}",
                        f"{st_e['rmse']:.4f}", f"{st_e['max_abs']:.4f}", ""])
    print(f"표 저장: {out_csv}")

    make_figure(pairs, session / "eval_fig.png")


if __name__ == "__main__":
    main()
