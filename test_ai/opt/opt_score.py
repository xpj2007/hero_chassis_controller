#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Score opt_metrics.py JSONs under several weight profiles (test-only).

Lower score is better. The weights/scales are explicit so the trade-off is
visible; this is a Pareto-style screen, not a claim of a single global optimum.

usage: opt_score.py results/base_cal.json [results/case2.json ...]
"""

import argparse
import json
import math
import sys


def g(d, *path, default=float("nan")):
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur if cur is not None else default


def metric(m, name):
    s = m.get("summary", {})
    if name == "static_a_mm":
        return s.get("static_a_mm")
    if name == "static_a_deg":
        return abs(s.get("static_a_deg") or float("nan"))
    if name == "static_b_mm":
        return s.get("static_b_mm")
    if name == "static_b_deg":
        return abs(s.get("static_b_deg") or float("nan"))
    if name == "static_b_odom_mm":
        return s.get("static_b_odom_mm")
    if name == "static_b_odom_deg":
        return abs(s.get("static_b_odom_deg") or float("nan"))
    if name == "static_b_err_mm":
        return s.get("static_b_err_mm")
    if name == "static_b_err_deg":
        return abs(s.get("static_b_err_deg") or float("nan"))
    if name == "static_a_err_mm":
        return s.get("static_a_err_mm")
    if name == "stop_low":
        return abs(s.get("stop_low_mm") or float("nan"))
    if name == "stop_high":
        return abs(s.get("stop_high_mm") or float("nan"))
    if name == "stop_lat":
        return abs(s.get("stop_lateral_max_mm") or float("nan"))
    if name.startswith("wz_"):
        # profile names are wz_0.5 / wz_1.0 / ...; opt_cal labels are wz_0p5_p/n
        tok = name[3:].replace(".", "p")
        vals = []
        for suf in ("_p", "_n"):
            key = "wz_" + tok + suf
            r = s.get("wz_yaw_ratio", {}).get(key)
            if r is None or not math.isfinite(r):
                r = s.get("wz_ratio", {}).get(key)
            if r is not None and math.isfinite(r):
                vals.append(abs(1.0 - r) * 100)
        return sum(vals) / len(vals) if vals else float("nan")
    if name.startswith("vx_") or name.startswith("vy_"):
        r = m.get("motion", {}).get(name, {}).get("ratio")
        return abs(1.0 - r) * 100 if r is not None and math.isfinite(r) else float("nan")
    if name == "circle":
        r = g(m, "circle", "yaw_ratio", default=float("nan"))
        return abs(1.0 - r) * 100 if math.isfinite(r) else float("nan")
    if name == "hs_track":
        errs = []
        for k, v in m.get("high_speed", {}).items():
            cmd = v.get("cmd", [0, 0, 0])
            gt = v.get("gt_body", [0, 0, 0])
            if k.startswith("vx"):
                errs.append(abs(gt[0] - cmd[0]))
            else:
                errs.append(abs(gt[1] - cmd[1]))
        return max(errs) * 100 if errs else float("nan")
    raise KeyError(name)


# name: (weight, normalizer) -- score contribution = weight * metric / normalizer
PROFILES = {
    "W1_static": {
        "static_a_mm": (2.0, 1.0),
        "static_a_deg": (1.0, 0.5),
        "static_b_mm": (1.0, 50.0),
        "static_b_deg": (1.0, 5.0),
        "static_b_err_mm": (1.0, 50.0),
        "static_b_err_deg": (1.0, 5.0),
        "stop_low": (1.0, 20.0),
    },
    "W2_yaw": {
        "wz_0.5": (1.0, 0.5),
        "wz_1.0": (1.0, 0.5),
        "wz_1.5": (1.0, 0.5),
        "wz_2.0": (1.0, 0.5),
        "circle": (2.0, 3.0),
        "static_b_err_deg": (1.0, 5.0),
    },
    "W3_speed": {
        "vx_1p5_p": (1.0, 1.0), "vx_1p5_n": (1.0, 1.0),
        "vx_2p0_p": (1.0, 1.0), "vx_2p0_n": (1.0, 1.0),
        "vx_2p5_p": (1.0, 1.0), "vx_2p5_n": (1.0, 1.0),
        "vy_1p5_p": (1.0, 1.0), "vy_1p5_n": (1.0, 1.0),
        "vy_2p0_p": (1.0, 1.0), "vy_2p0_n": (1.0, 1.0),
        "vy_2p5_p": (1.0, 1.0), "vy_2p5_n": (1.0, 1.0),
        "hs_track": (2.0, 0.5),
        "stop_high": (1.0, 500.0),
    },
    "W4_balanced": {
        "static_a_mm": (1.0, 1.0),
        "static_a_deg": (1.0, 0.5),
        "static_b_mm": (1.0, 50.0),
        "static_b_deg": (1.0, 5.0),
        "static_b_err_mm": (1.0, 50.0),
        "static_b_err_deg": (1.0, 5.0),
        "stop_low": (1.0, 20.0),
        "wz_0.5": (1.0, 0.5), "wz_1.0": (1.0, 0.5),
        "wz_1.5": (1.0, 0.5), "wz_2.0": (1.0, 0.5),
        "circle": (1.0, 3.0),
        "vx_1p5_p": (1.0, 1.0), "vx_1p5_n": (1.0, 1.0),
        "vx_2p0_p": (1.0, 1.0), "vx_2p0_n": (1.0, 1.0),
        "vx_2p5_p": (1.0, 1.0), "vx_2p5_n": (1.0, 1.0),
        "vy_1p5_p": (1.0, 1.0), "vy_1p5_n": (1.0, 1.0),
        "vy_2p0_p": (1.0, 1.0), "vy_2p0_n": (1.0, 1.0),
        "vy_2p5_p": (1.0, 1.0), "vy_2p5_n": (1.0, 1.0),
        "hs_track": (1.0, 0.5),
        "stop_high": (1.0, 500.0),
    },
}


def score(m, prof):
    num = den = 0.0
    detail = {}
    for name, (w, scale) in prof.items():
        try:
            v = metric(m, name)
        except KeyError:
            v = float("nan")
        if v is None or not math.isfinite(v):
            continue
        detail[name] = v
        num += w * v / scale
        den += w
    return (num / den if den else float("nan")), detail


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    args = ap.parse_args()
    runs = []
    for path in args.files:
        with open(path) as f:
            m = json.load(f)
        name = m.get("csv", path).split("/")[-1].replace(".csv", "")
        runs.append((name, m))
    print("=" * 110)
    print("run".ljust(18), " ".join(k.ljust(12) for k in PROFILES))
    print("-" * 110)
    for name, m in runs:
        row = []
        for pname, prof in PROFILES.items():
            s, _ = score(m, prof)
            row.append(("%.3f" % s).ljust(12) if math.isfinite(s) else "n/a".ljust(12))
        print(name.ljust(18), " ".join(row))
    print("-" * 110)
    # raw key metrics
    print("raw metrics:")
    keys = ["static_a_mm", "static_a_deg", "static_b_mm", "static_b_deg",
            "static_b_odom_mm", "static_b_odom_deg", "stop_lat", "wz_0.5", "wz_1.0",
            "wz_1.5", "wz_2.0", "vx_1p5_p", "vx_2p0_p", "vx_2p5_p",
            "vy_1p5_p", "vy_2p0_p", "vy_2p5_p", "circle"]
    print("run".ljust(18), " ".join(k.ljust(10) for k in keys))
    for name, m in runs:
        row = []
        for k in keys:
            try:
                v = metric(m, k)
                row.append(("%.2f" % v).ljust(10) if v is not None and math.isfinite(v) else "n/a".ljust(10))
            except KeyError:
                row.append("n/a".ljust(10))
        print(name.ljust(18), " ".join(row))
    print("=" * 110)


if __name__ == "__main__":
    sys.exit(main())
