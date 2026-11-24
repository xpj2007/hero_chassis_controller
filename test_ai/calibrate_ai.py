#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Original-rule calibration for hero_chassis_controller.

The original procedure (same math as the former scripts/calibrate.py):
  r : drive straight at constant vx, compare the controller odometry vx with
      Gazebo ground truth, r_new = r_old * v_gt / v_odom.
  k : rotate in place at constant wz, accumulate odometry yaw and Gazebo yaw
      (shortest-quaternion delta), k_new = k_old * odom_yaw / gt_yaw.

The world is expected to be already running (run_calib.sh starts it with
paused:=false); this node only publishes /cmd_vel and records.

usage:
  calibrate_ai.py --mode r --speed 0.3 --duration 10 --r-old 0.07625 --out ...
  calibrate_ai.py --mode k --speed 0.5 --duration 60  --k-old 0.4    --out ...
"""

import argparse
import json
import math
import os
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

WHEELS = ("left_front_wheel_joint", "left_back_wheel_joint",
          "right_front_wheel_joint", "right_back_wheel_joint")


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def shortest(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Recorder(object):
    def __init__(self):
        self.odom = []      # (stamp, x, y, yaw, vx, vy, wz)
        self.gt = []        # (recv, x, y, yaw)
        self.joint = []     # (stamp, lf, lb, rf, rb)
        self.hero_idx = None

    def clear(self):
        self.odom = []
        self.gt = []
        self.joint = []

    def odom_cb(self, msg):
        t = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        tw = msg.twist.twist
        self.odom.append((t, p.x, p.y, quat_to_yaw(msg.pose.pose.orientation),
                          tw.linear.x, tw.linear.y, tw.angular.z))

    def model_cb(self, msg):
        if self.hero_idx is None:
            try:
                self.hero_idx = list(msg.name).index("hero")
            except ValueError:
                return
        if self.hero_idx >= len(msg.pose):
            return
        p = msg.pose[self.hero_idx]
        self.gt.append((rospy.Time.now().to_sec(), p.position.x,
                        p.position.y, quat_to_yaw(p.orientation)))

    def joint_cb(self, msg):
        vel = dict(zip(msg.name, msg.velocity))
        if not all(n in vel for n in WHEELS):
            return
        self.joint.append((msg.header.stamp.to_sec(),
                           vel[WHEELS[0]], vel[WHEELS[1]],
                           vel[WHEELS[2]], vel[WHEELS[3]]))


def wait_for(pred, timeout, what):
    t0 = time.time()
    while not pred():
        if rospy.is_shutdown():
            raise RuntimeError("shutdown while waiting for " + what)
        if time.time() - t0 > timeout:
            raise RuntimeError("timeout waiting for " + what)
        time.sleep(0.05)


def interp(samples, t, col):
    if not samples:
        return None
    if t <= samples[0][0]:
        return samples[0][col]
    if t >= samples[-1][0]:
        return samples[-1][col]
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid][0] <= t:
            lo = mid
        else:
            hi = mid
    t0, t1 = samples[lo][0], samples[hi][0]
    if t1 <= t0:
        return samples[lo][col]
    a = (t - t0) / (t1 - t0)
    return samples[lo][col] * (1.0 - a) + samples[hi][col] * a


def yaw_accum(samples, t_from, t_to, yaw_col):
    """Accumulate shortest yaw deltas between consecutive samples in window."""
    acc = 0.0
    last = None
    for s in samples:
        if s[0] < t_from or s[0] > t_to:
            continue
        y = s[yaw_col]
        if last is not None:
            acc += shortest(y, last)
        last = y
    return acc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("r", "k"), required=True)
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--settle", type=float, default=5.0)
    ap.add_argument("--tail", type=float, default=1.0)
    ap.add_argument("--steady-start", type=float, default=1.0)
    ap.add_argument("--r-old", type=float, default=0.07625)
    ap.add_argument("--k-old", type=float, default=0.4)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    if args.speed is None:
        args.speed = 0.3 if args.mode == "r" else 0.5
    if args.duration is None:
        args.duration = 10.0 if args.mode == "r" else 60.0

    rospy.init_node("hero_calibrate_ai", anonymous=True)
    rec = Recorder()
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/odom", Odometry, rec.odom_cb, queue_size=200)
    rospy.Subscriber("/gazebo/model_states", ModelStates, rec.model_cb, queue_size=100)
    rospy.Subscriber("/joint_states", JointState, rec.joint_cb, queue_size=100)

    wait_for(lambda: cmd_pub.get_num_connections() > 0, 60.0, "/cmd_vel subscriber")
    wait_for(lambda: len(rec.odom) > 0, 30.0, "first /odom")
    wait_for(lambda: rec.hero_idx is not None, 30.0, "hero in /gazebo/model_states")

    # Let the spawn drop settle before the baseline (same as the original script).
    rospy.loginfo("settle %.1f s", args.settle)
    rospy.sleep(args.settle)
    rec.clear()
    rospy.sleep(0.2)

    cmd = Twist()
    if args.mode == "r":
        cmd.linear.x = args.speed
    else:
        cmd.angular.z = args.speed
    rate = rospy.Rate(50.0)
    rospy.loginfo("start %s: speed %.3f duration %.1f s", args.mode,
                  args.speed, args.duration)
    t0 = rospy.Time.now().to_sec()
    t_end = t0 + args.duration
    while rospy.Time.now().to_sec() < t_end and not rospy.is_shutdown():
        cmd_pub.publish(cmd)
        rate.sleep()
    t_cmd_end = rospy.Time.now().to_sec()
    zero = Twist()
    for _ in range(20):
        cmd_pub.publish(zero)
        rate.sleep()
    t_all_end = t_cmd_end + args.tail
    while rospy.Time.now().to_sec() < t_all_end and not rospy.is_shutdown():
        rate.sleep()

    result = {"mode": args.mode, "speed": args.speed,
              "duration": args.duration, "t0": t0, "t_cmd_end": t_cmd_end,
              "odom_samples": len(rec.odom), "gt_samples": len(rec.gt),
              "joint_samples": len(rec.joint)}
    if not rec.odom or not rec.gt:
        result["error"] = "missing odom/gt"
        print(json.dumps(result, indent=2))
        return 1

    if args.mode == "r":
        w1 = t0 + args.steady_start
        w2 = t_cmd_end - 0.5
        od = [s for s in rec.odom if w1 <= s[0] <= w2]
        jt = [s for s in rec.joint if w1 <= s[0] <= w2]
        if not od:
            result["error"] = "no odom samples in steady window"
            print(json.dumps(result, indent=2))
            return 1
        odom_vx = sum(s[4] for s in od) / len(od)
        x1 = interp(rec.gt, w1, 1)
        x2 = interp(rec.gt, w2, 1)
        y1 = interp(rec.gt, w1, 2)
        y2 = interp(rec.gt, w2, 2)
        gt_vx = math.hypot(x2 - x1, y2 - y1) / (w2 - w1)
        ratio = gt_vx / odom_vx if abs(odom_vx) > 1e-9 else float("nan")
        wheel_mean = ([sum(s[i] for s in jt) / len(jt) for i in (1, 2, 3, 4)]
                      if jt else [])
        wheel_abs_sum = (sum(abs(s[1]) + abs(s[2]) + abs(s[3]) + abs(s[4])
                             for s in jt) / len(jt)) if jt else float("nan")
        result.update({
            "window": [w1, w2],
            "odom_vx": odom_vx, "gt_vx": gt_vx,
            "ratio_vgt_over_vodom": ratio,
            "r_old": args.r_old,
            "r_new": args.r_old * ratio if math.isfinite(ratio) else None,
            "wheel_mean": wheel_mean,
            "wheel_abs_sum_mean": wheel_abs_sum,
        })
        if not jt or wheel_abs_sum < 0.1:
            result["warning"] = "wheel speeds ~0: the command may not have been applied"
    else:
        # Official original-rule accumulation: whole recording (command + tail).
        odom_yaw = yaw_accum(rec.odom, t0, t_cmd_end + args.tail, 3)
        gt_yaw = yaw_accum(rec.gt, t0, t_cmd_end + args.tail, 3)
        odom_yaw_cmd = yaw_accum(rec.odom, t0, t_cmd_end, 3)
        gt_yaw_cmd = yaw_accum(rec.gt, t0, t_cmd_end, 3)
        ratio = odom_yaw / gt_yaw if abs(gt_yaw) > 1e-6 else float("nan")
        ratio_cmd = (odom_yaw_cmd / gt_yaw_cmd
                     if abs(gt_yaw_cmd) > 1e-6 else float("nan"))
        jt = rec.joint
        wheel_abs_sum = (sum(abs(s[1]) + abs(s[2]) + abs(s[3]) + abs(s[4])
                             for s in jt) / len(jt)) if jt else float("nan")
        result.update({
            "odom_yaw": odom_yaw, "gt_yaw": gt_yaw,
            "ratio_odom_over_gt": ratio,
            "k_old": args.k_old,
            "k_new": args.k_old * ratio if math.isfinite(ratio) else None,
            "odom_yaw_cmd": odom_yaw_cmd, "gt_yaw_cmd": gt_yaw_cmd,
            "ratio_cmd": ratio_cmd,
            "k_new_cmd": (args.k_old * ratio_cmd
                          if math.isfinite(ratio_cmd) else None),
            "delta_deg": math.degrees(gt_yaw - odom_yaw),
            "wheel_abs_sum_mean": wheel_abs_sum,
        })
        if not jt or wheel_abs_sum < 0.1:
            result["warning"] = "wheel speeds ~0: the command may not have been applied"

    print(json.dumps(result, indent=2))
    if args.out:
        d = os.path.dirname(os.path.abspath(args.out))
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(args.out, "w") as f:
            json.dump(result, f, indent=2)
        rospy.loginfo("wrote %s", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
