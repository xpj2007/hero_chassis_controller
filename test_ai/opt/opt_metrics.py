#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Per-segment metrics for the optimization scenarios (test-only).

Reads test_ai/data/<run>.csv (written by record_scenarios.py) and computes:
  - motion segments vx_*/vy_*/wz_*: odom/GT body velocity, yaw ratio, r/k estimates
  - static_a/static_b: GT/odom drift and yaw
  - stop_*: post-motion displacement, lateral drift, free-wheel spin
  - circle: GT/odom yaw
No controller code is touched.

usage: opt_metrics.py DATA.csv OUT.json [--r-old 0.076309] [--k-old 0.390222]
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

WHEELS = ("w_lf", "w_lb", "w_rf", "w_rb")


def fnum(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            d = {"seg": r["seg"], "sim_t": fnum(r["sim_t"]),
                 "od_x": fnum(r["od_x"]), "od_y": fnum(r["od_y"]),
                 "od_yaw": fnum(r["od_yaw"]),
                 "od_vx": fnum(r["od_vx"]), "od_vy": fnum(r["od_vy"]),
                 "od_wz": fnum(r["od_wz"]),
                 "gt_x": fnum(r["gt_x"]), "gt_y": fnum(r["gt_y"]),
                 "gt_yaw": fnum(r["gt_yaw"]),
                 "gt_vx_w": fnum(r["gt_vx_w"]), "gt_vy_w": fnum(r["gt_vy_w"]),
                 "gt_wz": fnum(r["gt_wz"]),
                 "cmd_vx": fnum(r["cmd_vx"]), "cmd_vy": fnum(r["cmd_vy"]),
                 "cmd_wz": fnum(r["cmd_wz"]),
                 "pid_stamp": fnum(r["pid_stamp"])}
            d.update({w: fnum(r[w]) for w in WHEELS})
            rows.append(d)
    return rows


def groups(rows):
    out = []
    for r in rows:
        if not out or out[-1]["seg"] != r["seg"]:
            out.append({"seg": r["seg"], "rows": []})
        out[-1]["rows"].append(r)
    return [g for g in out if g["rows"]]


def arr(rs, key):
    return np.array([r[key] for r in rs], dtype=float)


def finite(a):
    return a[np.isfinite(a)]


def yaw_delta(rs, key):
    y = finite(arr(rs, key))
    if len(y) < 2:
        return float("nan")
    return float(np.unwrap(y)[-1] - np.unwrap(y)[0])


def gt_body(rs):
    yaw = np.nanmean(arr(rs, "gt_yaw"))
    vx = np.nanmean(arr(rs, "gt_vx_w"))
    vy = np.nanmean(arr(rs, "gt_vy_w"))
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy, yaw


def mean(x):
    x = finite(np.asarray(x, dtype=float))
    return float(np.mean(x)) if len(x) else float("nan")


def motion_metrics(g, r_old, k_old):
    rs = g["rows"]
    t0, t1 = rs[0]["sim_t"], rs[-1]["sim_t"]
    ws = [r for r in rs if r["sim_t"] >= t0 + 1.5]
    if len(ws) < 5:
        ws = rs
    vx_b, vy_b, yaw = gt_body(ws)
    od_vx, od_vy = mean([r["od_vx"] for r in ws]), mean([r["od_vy"] for r in ws])
    gt_wz, od_wz = mean([r["gt_wz"] for r in ws]), mean([r["od_wz"] for r in ws])
    cmd = [mean([r["cmd_vx"] for r in ws]), mean([r["cmd_vy"] for r in ws]),
           mean([r["cmd_wz"] for r in ws])]
    gt_yaw = yaw_delta(ws, "gt_yaw")
    od_yaw = yaw_delta(ws, "od_yaw")
    wheels = [mean([abs(r[w]) for r in ws]) for w in WHEELS]
    out = {"seg": g["seg"], "t0": t0, "t1": t1, "cmd": cmd,
           "gt_body": [vx_b, vy_b, gt_wz], "odom_body": [od_vx, od_vy, od_wz],
           "gt_yaw_delta": gt_yaw, "od_yaw_delta": od_yaw,
           "wheel_abs_mean": wheels, "wheel_max": max(wheels)}
    if g["seg"].startswith("vx"):
        out["ratio"] = od_vx / vx_b if abs(vx_b) > 1e-6 else float("nan")
        out["r_est"] = r_old * abs(vx_b) / abs(od_vx) if abs(od_vx) > 1e-6 else float("nan")
    elif g["seg"].startswith("vy"):
        out["ratio"] = od_vy / vy_b if abs(vy_b) > 1e-6 else float("nan")
        out["r_est"] = r_old * abs(vy_b) / abs(od_vy) if abs(od_vy) > 1e-6 else float("nan")
    elif g["seg"].startswith("wz"):
        out["ratio"] = od_wz / gt_wz if abs(gt_wz) > 1e-6 else float("nan")
        out["yaw_ratio"] = od_yaw / gt_yaw if abs(gt_yaw) > 1e-6 else float("nan")
        out["k_est"] = k_old * od_yaw / gt_yaw if abs(gt_yaw) > 1e-6 else float("nan")
    if g["seg"] in ("circle",):
        out["yaw_ratio"] = od_yaw / gt_yaw if abs(gt_yaw) > 1e-6 else float("nan")
        out["k_est"] = k_old * od_yaw / gt_yaw if abs(gt_yaw) > 1e-6 else float("nan")
    return out


def static_metrics(g):
    rs = g["rows"]
    gx = finite(arr(rs, "gt_x")); gy = finite(arr(rs, "gt_y"))
    ox = finite(arr(rs, "od_x")); oy = finite(arr(rs, "od_y"))
    gdx = 1000 * (gx[-1] - gx[0]) if len(gx) else float("nan")
    gdy = 1000 * (gy[-1] - gy[0]) if len(gy) else float("nan")
    odx = 1000 * (ox[-1] - ox[0]) if len(ox) else float("nan")
    ody = 1000 * (oy[-1] - oy[0]) if len(oy) else float("nan")
    gth = math.degrees(yaw_delta(rs, "gt_yaw"))
    odh = math.degrees(yaw_delta(rs, "od_yaw"))
    out = {"seg": g["seg"], "dur": rs[-1]["sim_t"] - rs[0]["sim_t"],
           "gt_d_mm": math.hypot(gdx, gdy), "gt_dx_mm": gdx, "gt_dy_mm": gdy,
           "gt_dyaw_deg": gth,
           "od_d_mm": math.hypot(odx, ody), "od_dx_mm": odx, "od_dy_mm": ody,
           "od_dyaw_deg": odh,
           "od_err_mm": math.hypot(odx - gdx, ody - gdy),
           "od_err_dx_mm": odx - gdx, "od_err_dy_mm": ody - gdy,
           "od_err_deg": odh - gth,
           "wheel_abs_sum": mean([sum(abs(r[w]) for w in WHEELS) for r in rs]),
           "pid_active": mean([1.0 if rs[0]["sim_t"] - r["pid_stamp"] < 0.02 else 0.0 for r in rs])}
    return out


def stop_metrics(g, prev_seg):
    rs = g["rows"]
    gx = finite(arr(rs, "gt_x")); gy = finite(arr(rs, "gt_y"))
    ox = finite(arr(rs, "od_x")); oy = finite(arr(rs, "od_y"))
    dx = 1000 * (gx[-1] - gx[0]) if len(gx) else float("nan")
    dy = 1000 * (gy[-1] - gy[0]) if len(gy) else float("nan")
    odx = 1000 * (ox[-1] - ox[0]) if len(ox) else float("nan")
    ody = 1000 * (oy[-1] - oy[0]) if len(oy) else float("nan")
    if prev_seg.startswith("vx"):
        lat, olat = dy, ody
    elif prev_seg.startswith("vy"):
        lat, olat = dx, odx
    else:
        lat, olat = math.hypot(dx, dy), math.hypot(odx, ody)
    return {"seg": g["seg"], "prev": prev_seg,
            "gt_d_mm": math.hypot(dx, dy), "gt_dx_mm": dx, "gt_dy_mm": dy,
            "lateral_mm": lat, "od_d_mm": math.hypot(odx, ody),
            "od_lateral_mm": olat,
            "gt_dyaw_deg": math.degrees(yaw_delta(rs, "gt_yaw")),
            "od_dyaw_deg": math.degrees(yaw_delta(rs, "od_yaw")),
            "wheel_abs_sum": mean([sum(abs(r[w]) for w in WHEELS) for r in rs])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--r-old", type=float, default=0.076309)
    ap.add_argument("--k-old", type=float, default=0.390222)
    args = ap.parse_args()
    rows = load(args.csv)
    if not rows:
        print("no rows", args.csv)
        return 1
    gs = groups(rows)
    result = {"csv": args.csv, "r_old": args.r_old, "k_old": args.k_old,
              "motion": {}, "static": {}, "stop": {}, "circle": None}
    prev = ""
    for g in gs:
        name = g["seg"]
        if name.startswith("static"):
            result["static"][name] = static_metrics(g)
        elif name.startswith("stop"):
            result["stop"][name] = stop_metrics(g, prev)
        elif name == "circle":
            result["circle"] = motion_metrics(g, args.r_old, args.k_old)
        else:
            result["motion"][name] = motion_metrics(g, args.r_old, args.k_old)
        if not name.startswith("stop"):
            prev = name
    # aggregate r/k estimates
    for prefix in ("vx", "vy", "wz"):
        est = [v.get("r_est" if prefix != "wz" else "k_est") for k, v in result["motion"].items()
               if k.startswith(prefix)]
        est = [e for e in est if e is not None and math.isfinite(e)]
        if est:
            result["r_est_" + prefix] = {"n": len(est), "mean": float(np.mean(est)),
                                         "median": float(np.median(est)),
                                         "min": float(np.min(est)), "max": float(np.max(est))}
    # high-speed summary
    hs = {}
    for k, v in result["motion"].items():
        if k.startswith(("vx_1p5", "vx_2p0", "vx_2p5", "vy_1p5", "vy_2p0", "vy_2p5")):
            hs[k] = {"ratio": v.get("ratio"), "gt_body": v["gt_body"],
                     "odom_body": v["odom_body"], "cmd": v["cmd"],
                     "wheel_max": v["wheel_max"]}
    result["high_speed"] = hs
    # static summary
    sa = result["static"].get("static_a")
    sb = result["static"].get("static_b")
    result["summary"] = {
        "static_a_mm": sa["gt_d_mm"] if sa else None,
        "static_a_deg": sa["gt_dyaw_deg"] if sa else None,
        "static_a_odom_mm": sa["od_d_mm"] if sa else None,
        "static_b_mm": sb["gt_d_mm"] if sb else None,
        "static_b_deg": sb["gt_dyaw_deg"] if sb else None,
        "static_b_odom_mm": sb["od_d_mm"] if sb else None,
        "static_b_odom_deg": sb["od_dyaw_deg"] if sb else None,
        "static_a_err_mm": sa["od_err_mm"] if sa else None,
        "static_a_err_deg": sa["od_err_deg"] if sa else None,
        "static_b_err_mm": sb["od_err_mm"] if sb else None,
        "static_b_err_deg": sb["od_err_deg"] if sb else None,
        "stop_lateral_max_mm": max([abs(v["lateral_mm"]) for v in result["stop"].values()],
                                   default=None),
        "stop_low_mm": max([abs(v["lateral_mm"]) for k, v in result["stop"].items()
                            if any(t in k for t in ("0p1", "0p5")) and v["prev"].startswith(("vx", "vy"))],
                           default=None),
        "stop_high_mm": max([abs(v["lateral_mm"]) for k, v in result["stop"].items()
                             if any(t in k for t in ("1p5", "2p0", "2p5")) and v["prev"].startswith(("vx", "vy"))],
                            default=None),
        "stop_wz_mm": max([abs(v["lateral_mm"]) for k, v in result["stop"].items()
                           if v["prev"].startswith("wz")], default=None),
        "wz_ratio": {k: v.get("ratio") for k, v in result["motion"].items()
                     if k.startswith("wz_")},
        "wz_yaw_ratio": {k: v.get("yaw_ratio") for k, v in result["motion"].items()
                         if k.startswith("wz_") and v.get("yaw_ratio") is not None},
        "r_vx": result.get("r_est_vx", {}).get("mean"),
        "r_vy": result.get("r_est_vy", {}).get("mean"),
        "k_yaw": result.get("r_est_wz", {}).get("mean"),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result["summary"], indent=2))
    print("r_est_vx", result.get("r_est_vx"))
    print("r_est_vy", result.get("r_est_vy"))
    print("k_est_wz", result.get("r_est_wz"))
    print("wrote", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
