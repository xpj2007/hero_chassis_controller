#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Analysis of hero chassis test CSV recordings.

Reads <run>.csv / <run>.json produced by record_scenarios.py and computes
odometry / control / drift metrics plus plots.  Pure numpy + matplotlib.

usage: analyze.py RUN [--dir DATA_DIR] [--outdir OUT_DIR] [--no-plots]
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# gear from config/controllers.yaml (overridable on the command line)
WHEEL_RADIUS = 0.07625
K_EFF = 0.389792

WHEELS = ("w_lf", "w_lb", "w_rf", "w_rb")


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def unwrap_nan(a):
    """np.unwrap that tolerates leading/trailing NaN without poisoning the rest."""
    a = np.asarray(a, dtype=float)
    ok = np.isfinite(a)
    out = np.full_like(a, np.nan)
    if ok.sum() == 0:
        return out
    idx = np.arange(len(a))
    filled = np.interp(idx, idx[ok], a[ok])
    u = np.unwrap(filled)
    u[~ok] = np.nan
    return u


def load_csv(path):
    with open(path) as f:
        r = csv.DictReader(f)
        cols = r.fieldnames
        raw = {c: [] for c in cols}
        for row in r:
            for c in cols:
                raw[c].append(row[c])
    data = {}
    for c in cols:
        if c == "seg":
            data[c] = np.array(raw[c], dtype=object)
        else:
            data[c] = np.array([float(x) if x not in ("", "nan") else np.nan
                                for x in raw[c]])
    return data


def deriv(t, y):
    """Central-difference derivative with NaN handling."""
    y = np.asarray(y, dtype=float)
    d = np.full_like(y, np.nan)
    ok = np.isfinite(y) & np.isfinite(t)
    idx = np.where(ok)[0]
    if len(idx) < 3:
        return d
    d[idx[1:-1]] = (y[idx[2:]] - y[idx[:-2]]) / (t[idx[2:]] - t[idx[:-2]])
    return d


def linfit(t, y):
    ok = np.isfinite(t) & np.isfinite(y)
    if ok.sum() < 3:
        return np.nan, np.nan
    p = np.polyfit(t[ok] - t[ok][0], y[ok], 1)
    return p[0], p[1]


def wheel_integral(tv, w, radius, k):
    """Ungated wheel odometry: integrate raw wheel velocities with the wheel yaw."""
    if len(tv) < 3:
        return 0.0, 0.0, 0.0
    vx = w.sum(1) * radius / 4.0
    vy = (-w[:, 0] + w[:, 1] + w[:, 2] - w[:, 3]) * radius / 4.0
    wz = (-w[:, 0] - w[:, 1] + w[:, 2] + w[:, 3]) * radius / (4.0 * k)
    x = y = yaw = 0.0
    for i in range(len(tv) - 1):
        dt = tv[i + 1] - tv[i]
        ym = yaw + 0.5 * wz[i] * dt
        x += (vx[i] * math.cos(ym) - vy[i] * math.sin(ym)) * dt
        y += (vx[i] * math.sin(ym) + vy[i] * math.cos(ym)) * dt
        yaw += wz[i] * dt
    return x, y, yaw


def segment_masks(labels):
    """Ordered unique segment labels (ignoring empty)."""
    out = []
    for lab in labels:
        if lab and lab not in out:
            out.append(lab)
    return out


def window(mask, t, a, b):
    return mask & (t >= a) & (t <= b)


def body_vel(d):
    yaw = d["gt_yaw"]
    vx = np.cos(yaw) * d["gt_vx_w"] + np.sin(yaw) * d["gt_vy_w"]
    vy = -np.sin(yaw) * d["gt_vx_w"] + np.cos(yaw) * d["gt_vy_w"]
    return vx, vy


def gt_vals(d):
    return d


def pid_age(data):
    if "pid_stamp" not in data:
        return np.full_like(data["sim_t"], np.nan)
    age = data["sim_t"] - data["pid_stamp"]
    age[~np.isfinite(age)] = np.nan
    return age


def summarize(data, meta, args):
    t = data["sim_t"]
    page = pid_age(data)
    wall = data["wall_t"]
    gt_yaw = unwrap_nan(data["gt_yaw"])
    od_yaw = unwrap_nan(data["od_yaw"])
    gvx, gvy = body_vel(data)
    out = {"meta": {}, "startup": {}, "static": [], "motion": [], "identification": {},
           "errors": {}}

    s0 = np.isfinite(t)
    if s0.sum() < 10:
        return out
    t0, t1 = t[np.isfinite(t)][0], t[np.isfinite(t)][-1]
    w0, w1 = wall[np.isfinite(wall)][0], wall[np.isfinite(wall)][-1]
    out["meta"] = {
        "rows": int(len(t)),
        "sim_span": float(t1 - t0),
        "wall_span": float(w1 - w0),
        "rtc": float((t1 - t0) / (w1 - w0)) if w1 > w0 else None,
        "odom_rate": float(np.nanmean(data["odom_n"]) / np.nanmean(np.diff(t))),
        "joint_rate": float(np.nanmean(data["joint_n"]) / np.nanmean(np.diff(t))),
        "gt_rate": float(np.nanmean(data["gt_n"]) / np.nanmean(np.diff(t))),
        "odom_age_mean": float(np.nanmean(data["odom_age"])),
        "odom_age_max": float(np.nanmax(data["odom_age"])),
        "scenario": meta.get("scenario"),
        "mode": args.mode,
        "wall_seconds": float(meta.get("end_wall", 0) - meta.get("start_wall", 0)),
    }

    # ---- startup drop transient (first rows, before first labelled segment)
    lab = data["seg"]
    pre = (t <= (t[np.isfinite(t)][0] + 8.0))
    okg = np.isfinite(data["gt_x"])
    if pre.sum() > 5 and okg.any():
        m = pre & np.isfinite(data["gt_x"])
        mm = pre & np.isfinite(data["od_x"])
        out["startup"] = {
            "gt_dz": float(data["gt_z"][m][-1] - data["gt_z"][m][0]),
            "gt_z_first": float(data["gt_z"][m][0]),
            "gt_z_settled": float(data["gt_z"][m][-1]),
            "gt_dx": float(data["gt_x"][m][-1] - data["gt_x"][m][0]),
            "gt_dy": float(data["gt_y"][m][-1] - data["gt_y"][m][0]),
            "od_dx": float(data["od_x"][mm][-1] - data["od_x"][mm][0]),
            "od_dy": float(data["od_y"][mm][-1] - data["od_y"][mm][0]),
            "max_abs_wheel": float(np.nanmax(np.abs(np.column_stack([data[w] for w in WHEELS])))),
            "first_sim": float(t[0]),
        }

    # ---- labelled segments
    out["segments"] = []
    for seg in segment_masks(lab):
        m = lab == seg
        if m.sum() < 3:
            continue
        st, et = t[m][0], t[m][-1]
        dur = et - st
        cmd = (float(np.mean(data["cmd_vx"][m])), float(np.mean(data["cmd_vy"][m])),
               float(np.mean(data["cmd_wz"][m])))
        sm = m & (t >= st + 0.6 * dur)
        if cmd != (0.0, 0.0, 0.0) and dur > 3.0:
            sm = m & (t >= st + 1.2) & (t <= et - 0.1)
        if args.mode == 1 and (abs(cmd[0]) + abs(cmd[1])) > 1e-9:
            oyaw = float(np.nanmean(od_yaw[sm]))
            cbx = math.cos(oyaw) * cmd[0] + math.sin(oyaw) * cmd[1]
            cby = -math.sin(oyaw) * cmd[0] + math.cos(oyaw) * cmd[1]
        else:
            cbx, cby = cmd[0], cmd[1]
        cmd_body = (cbx, cby, cmd[2])
        rec = {"label": seg, "t0": float(st), "t1": float(et), "dur": float(dur),
               "cmd": cmd, "cmd_body": cmd_body,
               "gt_dx": float(data["gt_x"][m][-1] - data["gt_x"][m][0]),
               "gt_dy": float(data["gt_y"][m][-1] - data["gt_y"][m][0]),
               "od_dx": float(data["od_x"][m][-1] - data["od_x"][m][0]),
               "od_dy": float(data["od_y"][m][-1] - data["od_y"][m][0]),
               "gt_dyaw": float(gt_yaw[m][-1] - gt_yaw[m][0]),
               "od_dyaw": float(od_yaw[m][-1] - od_yaw[m][0]),
               "gt_vx": float(np.nanmean(gvx[sm])), "gt_vy": float(np.nanmean(gvy[sm])),
               "gt_wz": float(np.nanmean(data["gt_wz"][sm])),
               "od_vx": float(np.nanmean(data["od_vx"][sm])),
               "od_vy": float(np.nanmean(data["od_vy"][sm])),
               "od_wz": float(np.nanmean(data["od_wz"][sm])),
               "w_mean": [float(np.nanmean(data[w][sm])) for w in WHEELS],
               "w_abs_mean": [float(np.nanmean(np.abs(data[w][sm]))) for w in WHEELS],
               "w_sum_abs": float(np.nanmean(sum(np.abs(data[w][sm]) for w in WHEELS))),
               "od_pose_span": float(math.hypot(data["od_x"][m][-1] - data["od_x"][m][0],
                                                data["od_y"][m][-1] - data["od_y"][m][0])),
               "gt_pose_span": float(math.hypot(data["gt_x"][m][-1] - data["gt_x"][m][0],
                                                data["gt_y"][m][-1] - data["gt_y"][m][0])),
               }
        # target wheel speeds from the body-frame command, config kinematics
        r, k = args.radius, args.k
        vx, vy, wz = cmd_body
        tgt = [(vx - vy - k * wz) / r, (vx + vy - k * wz) / r,
               (vx + vy + k * wz) / r, (vx - vy + k * wz) / r]
        rec["w_target"] = tgt
        rec["w_err"] = [rec["w_mean"][i] - tgt[i] for i in range(4)]
        if cmd == (0.0, 0.0, 0.0):
            # last 2 s are a true standstill even after a stop segment
            tm = m & (t >= et - 2.0)
            if tm.sum() > 2:
                rec["tail_gt_d"] = float(math.hypot(
                    data["gt_x"][tm][-1] - data["gt_x"][tm][0],
                    data["gt_y"][tm][-1] - data["gt_y"][tm][0]))
                rec["tail_od_d"] = float(math.hypot(
                    data["od_x"][tm][-1] - data["od_x"][tm][0],
                    data["od_y"][tm][-1] - data["od_y"][tm][0]))
                rec["tail_gt_dyaw"] = float(gt_yaw[tm][-1] - gt_yaw[tm][0])
                rec["tail_w_sum"] = float(np.nanmean(
                    sum(np.abs(data[w][tm]) for w in WHEELS)))
                rec["tail_gt_v"] = float(np.nanmean(np.hypot(gvx[tm], gvy[tm])))
            rec["type"] = "static"
            # drift over the whole segment and over the last part
            rec["gt_xy_drift"] = float(math.hypot(rec["gt_dx"], rec["gt_dy"]))
            rec["od_xy_drift"] = float(math.hypot(rec["od_dx"], rec["od_dy"]))
            rec["x_rate"], _ = linfit(t[m], data["gt_x"][m])
            rec["y_rate"], _ = linfit(t[m], data["gt_y"][m])
            rec["yaw_rate"], _ = linfit(t[m], gt_yaw[m])
            rec["od_x_rate"], _ = linfit(t[m], data["od_x"][m])
            rec["od_y_rate"], _ = linfit(t[m], data["od_y"][m])
            pa = page[m]
            if np.isfinite(pa).any():
                active = pa < 0.02
                rec["pid_active_frac"] = float(np.mean(active))
                rec["pid_age_max"] = float(np.nanmax(pa))
                if active.any():
                    rec["pid_out_abs_mean"] = float(np.nanmean(
                        np.abs(data["pid_out"][m][active])))
                    rec["pid_iterm_abs_mean"] = float(np.nanmean(
                        np.abs(data["pid_iterm"][m][active])))
                idle = pa > 0.2
                rec["idle_frac"] = float(np.mean(idle))
            rec["tail_xy_drift"] = rec.get("tail_gt_d", float("nan"))
            # windows
            rec["windows"] = []
            wlen = 20.0
            for a in np.arange(0.0, max(dur - wlen, 0.0) + 1e-9, wlen):
                wm = window(m, t, st + a, st + a + wlen)
                if wm.sum() < 10:
                    continue
                rec["windows"].append({
                    "a": float(a), "b": float(a + wlen),
                    "gt_d": float(math.hypot(data["gt_x"][wm][-1] - data["gt_x"][wm][0],
                                             data["gt_y"][wm][-1] - data["gt_y"][wm][0])),
                    "od_d": float(math.hypot(data["od_x"][wm][-1] - data["od_x"][wm][0],
                                             data["od_y"][wm][-1] - data["od_y"][wm][0])),
                    "gt_dyaw": float(gt_yaw[wm][-1] - gt_yaw[wm][0]),
                    "w_sum": float(np.nanmean(sum(np.abs(data[w][wm]) for w in WHEELS))),
                })
            wcols = np.column_stack([data[w][m] for w in WHEELS])
            wdx, wdy, wdyaw = wheel_integral(t[m], wcols, args.radius, args.k)
            rec["wheel_dx"] = float(wdx)
            rec["wheel_dy"] = float(wdy)
            rec["wheel_dyaw"] = float(wdyaw)
            out["static"].append(rec)
        else:
            rec["type"] = "motion"
            # direction error: world direction for mode 1, body direction for mode 0
            if (abs(cmd[0]) + abs(cmd[1])) > 1e-6 and \
               math.hypot(rec["gt_vx"], rec["gt_vy"]) > 0.02:
                if args.mode == 1:
                    gtd = math.atan2(float(np.nanmean(data["gt_vy_w"][sm])),
                                     float(np.nanmean(data["gt_vx_w"][sm])))
                    cd = math.atan2(cmd[1], cmd[0])
                    rec["dir_err"] = float(math.atan2(math.sin(gtd - cd),
                                                      math.cos(gtd - cd)))
                    rec["gt_world_dir"] = float(gtd)
                else:
                    gtd = math.atan2(rec["gt_vy"], rec["gt_vx"])
                    cd = math.atan2(cmd_body[1], cmd_body[0])
                    rec["dir_err"] = float(math.atan2(math.sin(gtd - cd),
                                                      math.cos(gtd - cd)))
                    rec["gt_world_dir"] = float(gtd)
            # rise time from GT velocity magnitude along commanded direction
            cmdv = np.array(cmd_body)
            if np.any(cmdv != 0):
                # pick a dominant axis for the response
                axis = int(np.argmax(np.abs(cmdv)))
                sig = [gvx, gvy, data["gt_wz"]][axis]
                steady = rec[["gt_vx", "gt_vy", "gt_wz"][axis]]
                mm = m & np.isfinite(sig)
                tt = t[mm]
                ss = sig[mm]
                if len(ss) > 5:
                    start = ss[0]
                    lo = start + 0.1 * (steady - start)
                    hi = start + 0.9 * (steady - start)
                    def cross(target):
                        d = (ss - target) * np.sign(steady - start)
                        idx = np.where(d >= 0)[0]
                        return float(tt[idx[0]] - st) if len(idx) else np.nan
                    rec["rise_10"] = cross(lo)
                    rec["rise_90"] = cross(hi)
                    rec["steady_axis"] = ["vx", "vy", "wz"][axis]
                    rec["steady_gt"] = float(steady)
                    rec["steady_err"] = float(cmdv[axis] - steady)
                    rec["overshoot"] = float((np.max(ss * np.sign(steady)) -
                                              abs(steady)) / abs(steady)) if steady else np.nan
            pa = page[m]
            if np.isfinite(pa).any():
                rec["pid_active_frac"] = float(np.mean(pa < 0.02))
                if np.isfinite(rec.get("pid_active_frac", np.nan)):
                    rec["pid_out_abs_mean"] = float(np.nanmean(np.abs(data["pid_out"][m])))
                    rec["pid_iterm_abs_mean"] = float(np.nanmean(np.abs(data["pid_iterm"][m])))
            out["motion"].append(rec)

    # ---- identification of rolling radius / yaw factor from steady wheel speeds
    ident = {}
    for rec in out["motion"]:
        cb = rec["cmd_body"]
        w = rec["w_mean"]
        if abs(cb[0]) > 2.0 * abs(cb[1]) and abs(cb[2]) < 0.15:
            sw = sum(w)
            if abs(sw) > 1e-6:
                ident.setdefault("radius_forward", []).append(
                    (rec["label"], 4.0 * rec["gt_vx"] / sw))
        if abs(cb[1]) > 2.0 * abs(cb[0]) and abs(cb[2]) < 0.15:
            sw = -w[0] + w[1] + w[2] - w[3]
            if abs(sw) > 1e-6:
                ident.setdefault("radius_strafe", []).append(
                    (rec["label"], 4.0 * rec["gt_vy"] / sw))
        if abs(cb[2]) > 0.1 and abs(cb[0]) + abs(cb[1]) < 0.05:
            sw = -w[0] - w[1] + w[2] + w[3]
            if abs(rec["gt_wz"]) > 1e-6:
                ident.setdefault("k_yaw", []).append(
                    (rec["label"], args.radius * sw / (4.0 * rec["gt_wz"])))
            if abs(rec["gt_dyaw"]) > 1e-6:
                ident.setdefault("yaw_ratio_od_gt", []).append(
                    (rec["label"], rec["od_dyaw"] / rec["gt_dyaw"]))
    out["identification"] = {}
    for key, vals in ident.items():
        arr = [v for _, v in vals if np.isfinite(v)]
        out["identification"][key] = {
            "per_segment": {lab: float(v) for lab, v in vals if np.isfinite(v)},
            "mean": float(np.mean(arr)) if arr else None,
            "median": float(np.median(arr)) if arr else None,
            "n": len(arr),
        }

    # ---- overall odometry error
    ok = np.isfinite(data["gt_x"]) & np.isfinite(data["od_x"])
    if ok.sum() > 10:
        ex = data["od_x"][ok] - data["gt_x"][ok]
        ey = data["od_y"][ok] - data["gt_y"][ok]
        eyaw = wrap(data["od_yaw"][ok] - data["gt_yaw"][ok])
        out["errors"] = {
            "n": int(ok.sum()),
            "rmse_xy": float(np.sqrt(np.mean(ex ** 2 + ey ** 2))),
            "max_xy": float(np.max(np.hypot(ex, ey))),
            "final_xy": float(math.hypot(ex[-1], ey[-1])),
            "rmse_yaw": float(np.sqrt(np.mean(eyaw ** 2))),
            "max_yaw": float(np.max(np.abs(eyaw))),
            "final_yaw": float(eyaw[-1]),
            "final_gt": [float(data["gt_x"][ok][-1]), float(data["gt_y"][ok][-1])],
            "final_od": [float(data["od_x"][ok][-1]), float(data["od_y"][ok][-1])],
            "gt_z_mean": float(np.nanmean(data["gt_z"][ok])),
        }
    return out


# --------------------------------------------------------------------------- plots
def plot_all(data, out, outdir):
    os.makedirs(outdir, exist_ok=True)
    t = data["sim_t"]
    gt_yaw = unwrap_nan(data["gt_yaw"])
    od_yaw = unwrap_nan(data["od_yaw"])
    gvx, gvy = body_vel(data)
    lab = data["seg"]

    # overview
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    ax[0].plot(t, data["gt_x"], label="GT x", lw=1.2)
    ax[0].plot(t, data["od_x"], label="odom x", lw=1.0, ls="--")
    ax[0].set_ylabel("x [m]"); ax[0].legend(loc="best"); ax[0].grid(alpha=.3)
    ax[1].plot(t, data["gt_y"], label="GT y", lw=1.2)
    ax[1].plot(t, data["od_y"], label="odom y", lw=1.0, ls="--")
    ax[1].set_ylabel("y [m]"); ax[1].legend(loc="best"); ax[1].grid(alpha=.3)
    ax[2].plot(t, np.degrees(gt_yaw), label="GT yaw", lw=1.2)
    ax[2].plot(t, np.degrees(od_yaw), label="odom yaw", lw=1.0, ls="--")
    ax[2].set_ylabel("yaw [deg]"); ax[2].legend(loc="best"); ax[2].grid(alpha=.3)
    ax[3].plot(t, gvx, label="GT body vx", lw=1.0)
    ax[3].plot(t, data["od_vx"], label="odom vx", lw=0.8, ls="--")
    ax[3].plot(t, data["cmd_vx"], label="cmd vx", lw=0.8, color="k")
    ax[3].set_ylabel("vx [m/s]"); ax[3].legend(loc="best", ncol=3); ax[3].grid(alpha=.3)
    ax[3].set_xlabel("sim time [s]")
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "overview.png"), dpi=120)
    plt.close(fig)

    # static drift panels
    stat = [s for s in out["static"] if s["dur"] >= 20]
    if stat:
        fig, ax = plt.subplots(2, len(stat), figsize=(6 * len(stat), 6.5),
                               squeeze=False)
        for j, s in enumerate(stat):
            m = (lab == s["label"])
            tt = t[m] - s["t0"]
            ax[0][j].plot(tt, (data["gt_x"][m] - data["gt_x"][m][0]) * 1000,
                          label="GT dx [mm]")
            ax[0][j].plot(tt, (data["gt_y"][m] - data["gt_y"][m][0]) * 1000,
                          label="GT dy [mm]")
            ax[0][j].plot(tt, (data["od_x"][m] - data["od_x"][m][0]) * 1000,
                          label="odom dx [mm]", ls="--")
            ax[0][j].plot(tt, (data["od_y"][m] - data["od_y"][m][0]) * 1000,
                          label="odom dy [mm]", ls="--")
            ax2 = ax[0][j].twinx()
            ax2.plot(tt, np.degrees(gt_yaw[m] - gt_yaw[m][0]), color="g", lw=0.8,
                     label="GT dyaw [deg]")
            ax2.set_ylabel("GT dyaw [deg]", color="g")
            ax[0][j].set_title("static: %s" % s["label"])
            ax[0][j].set_xlabel("t - t0 [s]"); ax[0][j].set_ylabel("displacement [mm]")
            ax[0][j].legend(loc="upper left", fontsize=7); ax[0][j].grid(alpha=.3)
            for w, c in zip(WHEELS, ("C0", "C1", "C2", "C3")):
                ax[1][j].plot(tt, data[w][m], label=w, lw=0.8, color=c)
            ax[1][j].set_title("wheel velocities (latch gate: sum |w| < %.2f)"
                               % 0.2)
            ax[1][j].set_xlabel("t - t0 [s]"); ax[1][j].set_ylabel("wheel [rad/s]")
            ax[1][j].legend(loc="upper right", fontsize=7, ncol=4)
            ax[1][j].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(outdir, "static.png"), dpi=120)
        plt.close(fig)

    # step responses
    steps = [s for s in out["motion"] if s["label"] in
             ("vx_p", "vx_n", "vy_p", "vy_n", "wz_p", "wz_n")]
    if steps:
        n = len(steps)
        ncol = 3
        nrow = int(math.ceil(n / ncol))
        fig, ax = plt.subplots(nrow, ncol, figsize=(5 * ncol, 3.4 * nrow),
                               squeeze=False)
        for i, s in enumerate(steps):
            a = ax[i // ncol][i % ncol]
            m = (lab == s["label"])
            tt = t[m] - s["t0"]
            axis = int(np.argmax(np.abs(s["cmd"])))
            sig = [gvx, gvy, data["gt_wz"]][axis]
            nam = ["vx [m/s]", "vy [m/s]", "wz [rad/s]"][axis]
            a.plot(tt, sig[m], label="GT", lw=1.2)
            a.plot(tt, [data["od_vx"], data["od_vy"], data["od_wz"]][axis][m],
                   label="odom", lw=1.0, ls="--")
            a.axhline(s["cmd_body"][axis], color="k", lw=0.8, label="cmd")
            a.set_title("%s (cmd %.2f)" % (s["label"], s["cmd_body"][axis]))
            a.set_xlabel("t - t0 [s]"); a.set_ylabel(nam); a.grid(alpha=.3)
            a.legend(fontsize=7)
        for k in range(n, nrow * ncol):
            ax[k // ncol][k % ncol].axis("off")
        fig.tight_layout(); fig.savefig(os.path.join(outdir, "steps.png"), dpi=120)
        plt.close(fig)

    # trajectory
    fig, ax = plt.subplots(1, 2, figsize=(12, 5.5))
    ax[0].plot(data["gt_x"], data["gt_y"], label="GT", lw=1.4)
    ax[0].plot(data["od_x"], data["od_y"], label="odom", lw=1.0, ls="--")
    ax[0].set_aspect("equal", adjustable="datalim")
    ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
    ax[0].set_title("trajectory"); ax[0].legend(); ax[0].grid(alpha=.3)
    ex = data["od_x"] - data["gt_x"]
    ey = data["od_y"] - data["gt_y"]
    ax[1].plot(t, np.hypot(ex, ey) * 1000, label="|p_odom - p_gt| [mm]")
    ax[1].plot(t, np.degrees(wrap(data["od_yaw"] - data["gt_yaw"])),
               label="yaw error [deg]", lw=0.9)
    ax[1].set_xlabel("sim time [s]"); ax[1].legend(); ax[1].grid(alpha=.3)
    ax[1].set_title("odometry error")
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "trajectory.png"), dpi=120)
    plt.close(fig)

    # speed sweep / wheel tracking
    sw = [s for s in out["motion"] if s["label"].startswith("sweep_")]
    if sw:
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
        cmds = [s["cmd"][0] for s in sw]
        gts = [s["gt_vx"] for s in sw]
        ods = [s["od_vx"] for s in sw]
        ax[0].plot(cmds, cmds, "k--", lw=0.8, label="ideal")
        ax[0].plot(cmds, gts, "o-", label="GT")
        ax[0].plot(cmds, ods, "s--", label="odom")
        ax[0].set_xlabel("commanded vx [m/s]"); ax[0].set_ylabel("measured vx [m/s]")
        ax[0].legend(); ax[0].grid(alpha=.3); ax[0].set_title("vx sweep")
        for s in sw:
            ax[1].plot([s["cmd"][0]] * 4, s["w_mean"], "o")
        ax[1].set_xlabel("commanded vx [m/s]"); ax[1].set_ylabel("wheel [rad/s]")
        ax[1].set_title("wheel speed vs command"); ax[1].grid(alpha=.3)
        fig.tight_layout(); fig.savefig(os.path.join(outdir, "sweep.png"), dpi=120)
        plt.close(fig)

    # yaw tracking
    wzs = [s for s in out["motion"] if abs(s["cmd"][2]) > 1e-6]
    if wzs:
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
        for s in wzs:
            m = (lab == s["label"])
            ax[0].plot(t[m] - s["t0"], np.degrees(gt_yaw[m] - gt_yaw[m][0]),
                       label="GT " + s["label"], lw=1.2)
            ax[0].plot(t[m] - s["t0"], np.degrees(od_yaw[m] - od_yaw[m][0]),
                       ls="--", label="odom " + s["label"], lw=1.0)
        ax[0].set_xlabel("t - t0 [s]"); ax[0].set_ylabel("dyaw [deg]")
        ax[0].legend(fontsize=7); ax[0].grid(alpha=.3); ax[0].set_title("yaw response")
        g = np.array([s["gt_dyaw"] for s in wzs])
        o = np.array([s["od_dyaw"] for s in wzs])
        ax[1].plot(g, g, "k--", lw=0.8)
        ax[1].plot(g, o, "o")
        for s, a, b in zip(wzs, g, o):
            ax[1].annotate(s["label"], (a, b), fontsize=7)
        ax[1].set_xlabel("GT dyaw [rad]"); ax[1].set_ylabel("odom dyaw [rad]")
        ax[1].grid(alpha=.3); ax[1].set_title("yaw accumulation")
        fig.tight_layout(); fig.savefig(os.path.join(outdir, "yaw.png"), dpi=120)
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", help="run name (csv/json under --dir)")
    ap.add_argument("--dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "data"))
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--radius", type=float, default=WHEEL_RADIUS)
    ap.add_argument("--k", type=float, default=K_EFF)
    ap.add_argument("--mode", type=int, default=1, choices=(0, 1),
                    help="1: /cmd_vel is odom/world frame; 0: body frame")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    csv_path = os.path.join(args.dir, args.run + ".csv")
    json_path = os.path.join(args.dir, args.run + ".json")
    outdir = args.outdir or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         "analysis", args.run)
    os.makedirs(outdir, exist_ok=True)
    data = load_csv(csv_path)
    meta = json.load(open(json_path)) if os.path.exists(json_path) else {}
    summ = summarize(data, meta, args)
    with open(os.path.join(outdir, "summary.json"), "w") as f:
        json.dump(summ, f, indent=2)
    if not args.no_plots:
        plot_all(data, summ, outdir)

    # console report
    print("=" * 78)
    print("RUN", args.run, " scenario", summ["meta"].get("scenario"))
    print("rows %d  sim %.1fs  wall %.1fs  RTC %.3f  rates: odom %.0f joint %.0f gt %.0f Hz"
          % (summ["meta"]["rows"], summ["meta"]["sim_span"], summ["meta"]["wall_span"],
             summ["meta"]["rtc"], summ["meta"]["odom_rate"],
             summ["meta"]["joint_rate"], summ["meta"]["gt_rate"]))
    print("startup: gt dz %.3f m, gt xy (%.4f, %.4f), odom xy (%.4f, %.4f), |w|max %.2f"
          % (summ["startup"].get("gt_dz", float("nan")),
             summ["startup"].get("gt_dx", float("nan")),
             summ["startup"].get("gt_dy", float("nan")),
             summ["startup"].get("od_dx", float("nan")),
             summ["startup"].get("od_dy", float("nan")),
             summ["startup"].get("max_abs_wheel", float("nan"))))
    print("-" * 78)
    print("%-10s %6s %7s %7s %8s %8s %8s %8s %7s" %
          ("seg", "dur", "gt_d", "od_d", "gt_vx", "od_vx", "gt_wz", "od_wz", "w_sum"))
    for s in summ["segments"]:
        print("%-10s %6.1f %7.3f %7.3f %8.3f %8.3f %8.3f %8.3f %7.3f" %
              (s["label"], s["dur"], s["gt_pose_span"], s["od_pose_span"],
               s["gt_vx"], s["od_vx"], s["gt_wz"], s["od_wz"], s["w_sum_abs"]))
    print("-" * 78)
    for s in summ["static"]:
        print("STATIC %-8s dur %.0fs  GT drift %.1f mm (dx %.1f dy %.1f)  odom %.1f mm  "
              "GT dyaw %.2f deg  drift rate (%.3f, %.3f) mm/s  yaw rate %.4f deg/s  w_sum %.3f"
              % (s["label"], s["dur"], s["gt_xy_drift"] * 1e3, s["gt_dx"] * 1e3,
                 s["gt_dy"] * 1e3, s["od_xy_drift"] * 1e3, math.degrees(s["gt_dyaw"]),
                 s["x_rate"] * 1e3, s["y_rate"] * 1e3,
                 math.degrees(s["yaw_rate"]), s["w_sum_abs"]))
        print("   ungated-wheel: d (%.1f,%.1f) mm  dyaw %.2f deg | gated odom d (%.1f,%.1f) mm dyaw %.2f" %
              (s.get("wheel_dx", float("nan")) * 1e3,
               s.get("wheel_dy", float("nan")) * 1e3,
               math.degrees(s.get("wheel_dyaw", float("nan"))),
               s["od_dx"] * 1e3, s["od_dy"] * 1e3, math.degrees(s["od_dyaw"])))
        if np.isfinite(s.get("tail_gt_d", float("nan"))):
            print("   tail 2s: GT %.2f mm odom %.2f mm dyaw %.3f deg w_sum %.3f | "
                  "pid_active %.2f pid_out %.3f" %
                  (s["tail_gt_d"] * 1e3, s["tail_od_d"] * 1e3,
                   math.degrees(s["tail_gt_dyaw"]), s["tail_w_sum"],
                   s.get("pid_active_frac", float("nan")),
                   s.get("pid_out_abs_mean", float("nan"))))
        for w in s.get("windows", []):
            print("   [%3.0f-%3.0f s] GT %.1f mm  odom %.1f mm  GT dyaw %.2f deg  w_sum %.3f"
                  % (w["a"], w["b"], w["gt_d"] * 1e3, w["od_d"] * 1e3,
                     math.degrees(w["gt_dyaw"]), w["w_sum"]))
    print("-" * 78)
    for s in summ["motion"]:
        print("%-10s cmd (%5.2f,%5.2f,%5.2f) body (%5.2f,%5.2f,%5.2f) gt (%6.3f,%6.3f,%6.3f) od (%6.3f,%6.3f,%6.3f) "
              "w_err %s rise90 %.2fs" %
              (s["label"], s["cmd"][0], s["cmd"][1], s["cmd"][2],
               s["cmd_body"][0], s["cmd_body"][1], s["cmd_body"][2],
               s["gt_vx"], s["gt_vy"], s["gt_wz"], s["od_vx"], s["od_vy"], s["od_wz"],
               "[" + ",".join("%+.2f" % e for e in s["w_err"]) + "]",
               s.get("rise_90", float("nan"))))
    print("-" * 78)
    for k, v in summ["identification"].items():
        print("ident %-18s mean %s median %s n=%d  %s" %
              (k, ("%.5f" % v["mean"]) if v["mean"] else "nan",
               ("%.5f" % v["median"]) if v["median"] else "nan", v["n"],
               ", ".join("%s=%.5f" % (a, b) for a, b in v["per_segment"].items())))
    print("errors:", json.dumps(summ["errors"], indent=None))
    print("outdir", outdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
