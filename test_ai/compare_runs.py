#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compare several analyzed runs (analysis/<run>/summary.json).

usage: compare_runs.py label=run [label=run ...] [--outdir DIR]
"""

import argparse
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import hero_chassis_controller.test.static_window as static_window

HERE = os.path.dirname(os.path.abspath(__file__))
ANALYSIS = os.path.join(HERE, "analysis")


def load(run):
    p = os.path.join(ANALYSIS, run, "summary.json")
    return json.load(open(p))


def pick_static(summ):
    cands = [s for s in summ.get("static", []) if s["dur"] >= 30]
    if not cands:
        cands = summ.get("static", [])
    return max(cands, key=lambda s: s["dur"]) if cands else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", help="label=run")
    ap.add_argument("--outdir", default=os.path.join(ANALYSIS, "compare"))
    ap.add_argument("--seg", default="static_a")
    ap.add_argument("--win", type=float, default=60.0)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    rows = []
    for item in args.runs:
        if "=" not in item:
            print("bad arg", item)
            return 2
        label, run = item.split("=", 1)
        s = load(run)
        st = static_window.metrics(run, args.seg, args.win)
        if st is not None:
            st["label"] = args.seg
            st["dur"] = args.win
        rows.append({"label": label, "run": run, "summ": s, "static": st})

    print("=" * 100)
    print("%-12s %-9s %6s %8s %8s %8s %9s %7s %7s %7s" %
          ("run", "scenario", "dur", "GT drift", "GT dyaw", "odom d", "odom yawE",
           "pidOn", "|out|", "w_sum"))
    for r in rows:
        st = r["static"]
        if st is None:
            print("%-12s %-9s  no static segment" % (r["label"], r["summ"]["meta"].get("scenario")))
            continue
        print("%-12s %-9s %6.0f %7.1fmm %7.2f° %7.1fmm %8.2f° %7.2f %7.3f %7.3f" %
              (r["label"], r["summ"]["meta"].get("scenario"), st["dur"],
               st["gt_d"] * 1e3, math.degrees(st["gt_dyaw"]),
               st["od_d"] * 1e3,
               math.degrees(st["od_dyaw"] - st["gt_dyaw"]),
               st.get("pid_active", float("nan")),
               st.get("pid_out", float("nan")),
               st.get("w_sum", float("nan"))))

    print("-" * 100)
    print("%-12s %-9s %10s %10s %10s %10s" %
          ("run", "scenario", "final_xy", "max_xy", "final_yaw", "rmse_xy"))
    for r in rows:
        e = r["summ"].get("errors", {})
        print("%-12s %-9s %9.1fmm %9.1fmm %9.2f° %9.1fmm" %
              (r["label"], r["summ"]["meta"].get("scenario"),
               e.get("final_xy", float("nan")) * 1e3,
               e.get("max_xy", float("nan")) * 1e3,
               math.degrees(e.get("final_yaw", float("nan"))),
               e.get("rmse_xy", float("nan")) * 1e3))
    print("-" * 100)
    common = ["vx_p", "vx_n", "vy_p", "vy_n", "wz_p", "wz_n"]
    print("steady-state (GT) per common step:")
    print("%-12s %s" % ("run", " ".join("%-22s" % c for c in common)))
    for r in rows:
        by = {s["label"]: s for s in r["summ"].get("motion", [])}
        cells = []
        for c in common:
            s = by.get(c)
            if not s:
                cells.append("%-22s" % "-")
            else:
                cells.append("%-22s" % ("%.3f/%.3f" % (s["gt_vx"] if c.startswith("vx") else
                                                       s["gt_vy"] if c.startswith("vy") else s["gt_wz"],
                                                       s.get("steady_err", float("nan")))))
        print("%-12s %s" % (r["label"], " ".join(cells)))

    # bar chart
    lab = [r["label"] for r in rows]
    gtd = [r["static"]["gt_d"] * 1e3 if r["static"] else 0 for r in rows]
    odd = [r["static"]["od_d"] * 1e3 if r["static"] else 0 for r in rows]
    x = np.arange(len(lab))
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    ax[0].bar(x - 0.2, gtd, 0.4, label="GT drift")
    ax[0].bar(x + 0.2, odd, 0.4, label="odom drift")
    ax[0].set_xticks(x); ax[0].set_xticklabels(lab, rotation=15)
    ax[0].set_ylabel("drift over static segment [mm]")
    ax[0].set_title("standstill drift")
    ax[0].legend(); ax[0].grid(alpha=.3, axis="y")
    yd = [math.degrees(r["static"]["gt_dyaw"]) if r["static"] else 0 for r in rows]
    ax[1].bar(x, yd, 0.5)
    ax[1].set_xticks(x); ax[1].set_xticklabels(lab, rotation=15)
    ax[1].set_ylabel("GT yaw drift [deg]")
    ax[1].set_title("standstill yaw drift")
    ax[1].grid(alpha=.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(args.outdir, "compare_static.png"), dpi=120)
    print("plot", os.path.join(args.outdir, "compare_static.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
