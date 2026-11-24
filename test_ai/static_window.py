#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fixed-window standstill metrics for A/B sessions.

usage: static_window.py RUN [RUN ...] [--seg static_a] [--win 60]
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
WHEELS = ("w_lf", "w_lb", "w_rf", "w_rb")
R = 0.07625
K = 0.389792


def load(run):
    p = os.path.join(DATA, run + ".csv")
    with open(p) as f:
        r = csv.DictReader(f)
        cols = r.fieldnames
        raw = {c: [] for c in cols}
        for row in r:
            for c in cols:
                raw[c].append(row[c])
    d = {}
    for c in cols:
        if c == "seg":
            d[c] = np.array(raw[c], dtype=object)
        else:
            d[c] = np.array([float(x) if x not in ("", "nan") else np.nan
                             for x in raw[c]])
    return d


def unwrap_nan(a):
    a = np.asarray(a, float)
    ok = np.isfinite(a)
    if ok.sum() == 0:
        return a
    u = np.unwrap(np.interp(np.arange(len(a)), np.arange(len(a))[ok], a[ok]))
    out = np.full_like(a, np.nan)
    out[ok] = u[ok]
    return out


def wheel_integral(tv, w):
    vx = w.sum(1) * R / 4.0
    vy = (-w[:, 0] + w[:, 1] + w[:, 2] - w[:, 3]) * R / 4.0
    wz = (-w[:, 0] - w[:, 1] + w[:, 2] + w[:, 3]) * R / (4.0 * K)
    x = y = yaw = 0.0
    for i in range(len(tv) - 1):
        dt = tv[i + 1] - tv[i]
        ym = yaw + 0.5 * wz[i] * dt
        x += (vx[i] * math.cos(ym) - vy[i] * math.sin(ym)) * dt
        y += (vx[i] * math.sin(ym) + vy[i] * math.cos(ym)) * dt
        yaw += wz[i] * dt
    return x, y, yaw


def metrics(run, seg, win):
    d = load(run)
    t = d["sim_t"]
    lab = d["seg"]
    m = lab == seg
    if m.sum() < 10:
        return None
    i = np.where(m)[0]
    t0 = t[i][0]
    m = m & (t <= t0 + win)
    i = np.where(m)[0]
    if len(i) < 10:
        return None
    w = np.column_stack([d[k][i] for k in WHEELS])
    wdx, wdy, wdyaw = wheel_integral(t[i], w)
    gy = unwrap_nan(d["gt_yaw"])
    oy = unwrap_nan(d["od_yaw"])
    page = t - d["pid_stamp"]
    active = page[i] < 0.02
    out = {
        "run": run, "seg": seg, "n": len(i),
        "t0": float(t0), "t1": float(t[i][-1]),
        "gt_dx": float(d["gt_x"][i][-1] - d["gt_x"][i][0]),
        "gt_dy": float(d["gt_y"][i][-1] - d["gt_y"][i][0]),
        "gt_dyaw": float(gy[i][-1] - gy[i][0]),
        "od_dx": float(d["od_x"][i][-1] - d["od_x"][i][0]),
        "od_dy": float(d["od_y"][i][-1] - d["od_y"][i][0]),
        "od_dyaw": float(oy[i][-1] - oy[i][0]),
        "wheel_dx": float(wdx), "wheel_dy": float(wdy), "wheel_dyaw": float(wdyaw),
        "w_sum": float(np.mean(np.abs(w).sum(1))),
        "pid_active": float(np.mean(active)) if np.isfinite(page[i]).any() else float("nan"),
        "pid_out": float(np.mean(np.abs(d["pid_out"][i][active]))) if active.any() else 0.0,
        "tail_gt": float(math.hypot(d["gt_x"][i][-1] - d["gt_x"][i][-2],
                                   d["gt_y"][i][-1] - d["gt_y"][i][-2])),
    }
    out["gt_d"] = float(math.hypot(out["gt_dx"], out["gt_dy"]))
    out["od_d"] = float(math.hypot(out["od_dx"], out["od_dy"]))
    out["wheel_d"] = float(math.hypot(out["wheel_dx"], out["wheel_dy"]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--seg", default="static_a")
    ap.add_argument("--win", type=float, default=60.0)
    args = ap.parse_args()
    rows = []
    for run in args.runs:
        m = metrics(run, args.seg, args.win)
        if m is None:
            print(run, "no segment", args.seg)
            continue
        rows.append(m)
        print("%-9s %-8s %5.1fs | GT d %6.1f mm (%6.1f,%6.1f) dyaw %6.2f | "
              "odom d %6.1f mm (%6.1f,%6.1f) dyaw %6.2f | wheel d %6.1f dyaw %6.2f | "
              "w_sum %.3f pidOn %.2f |out| %.3f"
              % (m["run"], m["seg"], m["t1"] - m["t0"],
                 m["gt_d"] * 1e3, m["gt_dx"] * 1e3, m["gt_dy"] * 1e3,
                 math.degrees(m["gt_dyaw"]),
                 m["od_d"] * 1e3, m["od_dx"] * 1e3, m["od_dy"] * 1e3,
                 math.degrees(m["od_dyaw"]),
                 m["wheel_d"] * 1e3, math.degrees(m["wheel_dyaw"]),
                 m["w_sum"], m["pid_active"], m["pid_out"]))
    print("-" * 120)
    print("%-9s %8s %8s %8s %8s %8s" %
          ("run", "GT d", "GT dyaw", "odom d", "wheel d", "w_sum"))
    for m in rows:
        print("%-9s %7.1f %7.2f %7.1f %7.1f %7.3f" %
              (m["run"], m["gt_d"] * 1e3, math.degrees(m["gt_dyaw"]),
               m["od_d"] * 1e3, m["wheel_d"] * 1e3, m["w_sum"]))


if __name__ == "__main__":
    sys.exit(main())
