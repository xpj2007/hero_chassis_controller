#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Hero chassis long-run data recorder / scenario driver (test scaffolding).

Subscribes (read only):
  /odom                 nav_msgs/Odometry   -- controller output
  /joint_states         sensor_msgs/JointState -- wheel velocities, 200 Hz
  /gazebo/model_states  gazebo_msgs/ModelStates -- physical ground truth
Publishes:
  /cmd_vel              geometry_msgs/Twist

Writes one CSV row per 5 ms (200 Hz) plus a JSON with scenario markers.
The world is started paused; this node calls /gazebo/unpause_physics as soon
as the subscriptions are up, so the spawn-drop transient is recorded.
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import rospy
from control_msgs.msg import PidState
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

WHEEL_NAMES = ("left_front_wheel_joint", "left_back_wheel_joint",
               "right_front_wheel_joint", "right_back_wheel_joint")

COLUMNS = [
    "wall_t", "sim_t",
    "odom_stamp", "odom_age", "odom_n", "joint_n", "gt_n",
    "gt_x", "gt_y", "gt_z", "gt_yaw",
    "gt_vx_w", "gt_vy_w", "gt_vz_w", "gt_wz",
    "od_x", "od_y", "od_yaw", "od_vx", "od_vy", "od_wz",
    "w_lf", "w_lb", "w_rf", "w_rb",
    "cmd_vx", "cmd_vy", "cmd_wz",
    "pid_stamp", "pid_err", "pid_pterm", "pid_iterm", "pid_dterm", "pid_out",
    "seg",
]

# name: (settle, segments)  segment = (label, duration_s, (vx, vy, wz))
SCENARIOS = {
    "smoke": (3.0, [
        ("static", 20.0, (0.0, 0.0, 0.0)),
        ("vx", 5.0, (0.3, 0.0, 0.0)),
        ("stop", 5.0, (0.0, 0.0, 0.0)),
    ]),
    "static": (8.0, [
        ("static", 180.0, (0.0, 0.0, 0.0)),
    ]),
    "steps": (8.0, [
        ("static", 30.0, (0.0, 0.0, 0.0)),
        ("vx_p", 10.0, (0.3, 0.0, 0.0)), ("stop1", 5.0, (0.0, 0.0, 0.0)),
        ("vx_n", 10.0, (-0.3, 0.0, 0.0)), ("stop2", 5.0, (0.0, 0.0, 0.0)),
        ("vy_p", 10.0, (0.0, 0.3, 0.0)), ("stop3", 5.0, (0.0, 0.0, 0.0)),
        ("vy_n", 10.0, (0.0, -0.3, 0.0)), ("stop4", 5.0, (0.0, 0.0, 0.0)),
        ("wz_p", 10.0, (0.0, 0.0, 0.5)), ("stop5", 5.0, (0.0, 0.0, 0.0)),
        ("wz_n", 10.0, (0.0, 0.0, -0.5)), ("stop6", 5.0, (0.0, 0.0, 0.0)),
    ]),
    "main": (8.0, [
        ("static_a", 120.0, (0.0, 0.0, 0.0)),
        ("vx_p", 10.0, (0.3, 0.0, 0.0)), ("stop1", 5.0, (0.0, 0.0, 0.0)),
        ("vx_n", 10.0, (-0.3, 0.0, 0.0)), ("stop2", 5.0, (0.0, 0.0, 0.0)),
        ("vy_p", 10.0, (0.0, 0.3, 0.0)), ("stop3", 5.0, (0.0, 0.0, 0.0)),
        ("vy_n", 10.0, (0.0, -0.3, 0.0)), ("stop4", 5.0, (0.0, 0.0, 0.0)),
        ("wz_p", 10.0, (0.0, 0.0, 0.5)), ("stop5", 5.0, (0.0, 0.0, 0.0)),
        ("wz_n", 10.0, (0.0, 0.0, -0.5)), ("stop6", 5.0, (0.0, 0.0, 0.0)),
        ("sweep_0p1", 6.0, (0.1, 0.0, 0.0)), ("s1", 3.0, (0.0, 0.0, 0.0)),
        ("sweep_0p3", 6.0, (0.3, 0.0, 0.0)), ("s2", 3.0, (0.0, 0.0, 0.0)),
        ("sweep_0p5", 6.0, (0.5, 0.0, 0.0)), ("s3", 3.0, (0.0, 0.0, 0.0)),
        ("sweep_0p8", 6.0, (0.8, 0.0, 0.0)), ("s4", 3.0, (0.0, 0.0, 0.0)),
        ("sweep_1p2", 6.0, (1.2, 0.0, 0.0)), ("s5", 3.0, (0.0, 0.0, 0.0)),
        # world-frame square, mode 1 only: 4 sides along +x +y -x -y
        ("ws_head", 1.5708, (0.0, 0.0, 1.0)), ("ws_s1", 5.0, (0.3, 0.0, 0.0)),
        ("wt1", 1.5708, (0.0, 0.0, 1.0)), ("ws_s2", 5.0, (0.0, 0.3, 0.0)),
        ("wt2", 1.5708, (0.0, 0.0, 1.0)), ("ws_s3", 5.0, (-0.3, 0.0, 0.0)),
        ("wt3", 1.5708, (0.0, 0.0, 1.0)), ("ws_s4", 5.0, (0.0, -0.3, 0.0)),
        ("circle", 25.0, (0.3, 0.0, 0.5)),
        ("static_b", 120.0, (0.0, 0.0, 0.0)),
    ]),
    "mode0": (8.0, [
        ("static_a", 60.0, (0.0, 0.0, 0.0)),
        ("vx_p", 10.0, (0.3, 0.0, 0.0)), ("stop1", 5.0, (0.0, 0.0, 0.0)),
        ("vx_n", 10.0, (-0.3, 0.0, 0.0)), ("stop2", 5.0, (0.0, 0.0, 0.0)),
        ("vy_p", 10.0, (0.0, 0.3, 0.0)), ("stop3", 5.0, (0.0, 0.0, 0.0)),
        ("vy_n", 10.0, (0.0, -0.3, 0.0)), ("stop4", 5.0, (0.0, 0.0, 0.0)),
        ("wz_p", 10.0, (0.0, 0.0, 0.5)), ("stop5", 5.0, (0.0, 0.0, 0.0)),
        ("wz_n", 10.0, (0.0, 0.0, -0.5)), ("stop6", 5.0, (0.0, 0.0, 0.0)),
        # body-frame square: forward 1.5 m then 90 deg, x4
        ("bs_s1", 5.0, (0.3, 0.0, 0.0)), ("bt1", 1.5708, (0.0, 0.0, 1.0)),
        ("bs_s2", 5.0, (0.3, 0.0, 0.0)), ("bt2", 1.5708, (0.0, 0.0, 1.0)),
        ("bs_s3", 5.0, (0.3, 0.0, 0.0)), ("bt3", 1.5708, (0.0, 0.0, 1.0)),
        ("bs_s4", 5.0, (0.3, 0.0, 0.0)), ("bt4", 1.5708, (0.0, 0.0, 1.0)),
        ("circle", 25.0, (0.3, 0.0, 0.5)),
        ("static_b", 60.0, (0.0, 0.0, 0.0)),
    ]),
    "static_short": (8.0, [
        ("static_a", 60.0, (0.0, 0.0, 0.0)),
    ]),
    "stale": (8.0, [
        ("warm", 3.0, (0.0, 0.0, 0.0)),
        ("stale_cmd", 10.0, (0.3, 0.0, 0.0)),  # published once only
        ("zero", 5.0, (0.0, 0.0, 0.0)),
    ]),
    "mode0_short": (8.0, [
        ("static_a", 30.0, (0.0, 0.0, 0.0)),
        ("vx_p", 8.0, (0.3, 0.0, 0.0)), ("stop1", 3.0, (0.0, 0.0, 0.0)),
        ("vy_p", 8.0, (0.0, 0.3, 0.0)), ("stop2", 3.0, (0.0, 0.0, 0.0)),
        ("wz_p", 8.0, (0.0, 0.0, 0.5)), ("stop3", 3.0, (0.0, 0.0, 0.0)),
        ("bs_s1", 4.0, (0.3, 0.0, 0.0)), ("bt1", 1.5708, (0.0, 0.0, 1.0)),
        ("bs_s2", 4.0, (0.3, 0.0, 0.0)), ("bt2", 1.5708, (0.0, 0.0, 1.0)),
        ("bs_s3", 4.0, (0.3, 0.0, 0.0)), ("bt3", 1.5708, (0.0, 0.0, 1.0)),
        ("bs_s4", 4.0, (0.3, 0.0, 0.0)), ("bt4", 1.5708, (0.0, 0.0, 1.0)),
        ("circle", 20.0, (0.3, 0.0, 0.5)),
        ("static_b", 30.0, (0.0, 0.0, 0.0)),
    ]),
    # --- optimization-only scenarios (test harness, controller untouched) ---
    # long multi-speed sweep: straight/strafe/wz both directions, up to 2.5 m/s
    "opt_cal": (8.0, [
        ("static_a", 30.0, (0.0, 0.0, 0.0)),
        ("vx_0p1_p", 8.0, (0.1, 0.0, 0.0)), ("stop_vx_0p1_p", 2.0, (0.0, 0.0, 0.0)),
        ("vx_0p1_n", 8.0, (-0.1, 0.0, 0.0)), ("stop_vx_0p1_n", 2.0, (0.0, 0.0, 0.0)),
        ("vx_0p5_p", 8.0, (0.5, 0.0, 0.0)), ("stop_vx_0p5_p", 2.0, (0.0, 0.0, 0.0)),
        ("vx_0p5_n", 8.0, (-0.5, 0.0, 0.0)), ("stop_vx_0p5_n", 2.0, (0.0, 0.0, 0.0)),
        ("vx_1p0_p", 8.0, (1.0, 0.0, 0.0)), ("stop_vx_1p0_p", 2.0, (0.0, 0.0, 0.0)),
        ("vx_1p0_n", 8.0, (-1.0, 0.0, 0.0)), ("stop_vx_1p0_n", 2.0, (0.0, 0.0, 0.0)),
        ("vx_1p5_p", 8.0, (1.5, 0.0, 0.0)), ("stop_vx_1p5_p", 2.0, (0.0, 0.0, 0.0)),
        ("vx_1p5_n", 8.0, (-1.5, 0.0, 0.0)), ("stop_vx_1p5_n", 2.0, (0.0, 0.0, 0.0)),
        ("vx_2p0_p", 8.0, (2.0, 0.0, 0.0)), ("stop_vx_2p0_p", 2.0, (0.0, 0.0, 0.0)),
        ("vx_2p0_n", 8.0, (-2.0, 0.0, 0.0)), ("stop_vx_2p0_n", 2.0, (0.0, 0.0, 0.0)),
        ("vx_2p5_p", 8.0, (2.5, 0.0, 0.0)), ("stop_vx_2p5_p", 2.0, (0.0, 0.0, 0.0)),
        ("vx_2p5_n", 8.0, (-2.5, 0.0, 0.0)), ("stop_vx_2p5_n", 2.0, (0.0, 0.0, 0.0)),
        ("vy_0p1_p", 8.0, (0.0, 0.1, 0.0)), ("stop_vy_0p1_p", 2.0, (0.0, 0.0, 0.0)),
        ("vy_0p1_n", 8.0, (0.0, -0.1, 0.0)), ("stop_vy_0p1_n", 2.0, (0.0, 0.0, 0.0)),
        ("vy_0p5_p", 8.0, (0.0, 0.5, 0.0)), ("stop_vy_0p5_p", 2.0, (0.0, 0.0, 0.0)),
        ("vy_0p5_n", 8.0, (0.0, -0.5, 0.0)), ("stop_vy_0p5_n", 2.0, (0.0, 0.0, 0.0)),
        ("vy_1p0_p", 8.0, (0.0, 1.0, 0.0)), ("stop_vy_1p0_p", 2.0, (0.0, 0.0, 0.0)),
        ("vy_1p0_n", 8.0, (0.0, -1.0, 0.0)), ("stop_vy_1p0_n", 2.0, (0.0, 0.0, 0.0)),
        ("vy_1p5_p", 8.0, (0.0, 1.5, 0.0)), ("stop_vy_1p5_p", 2.0, (0.0, 0.0, 0.0)),
        ("vy_1p5_n", 8.0, (0.0, -1.5, 0.0)), ("stop_vy_1p5_n", 2.0, (0.0, 0.0, 0.0)),
        ("vy_2p0_p", 8.0, (0.0, 2.0, 0.0)), ("stop_vy_2p0_p", 2.0, (0.0, 0.0, 0.0)),
        ("vy_2p0_n", 8.0, (0.0, -2.0, 0.0)), ("stop_vy_2p0_n", 2.0, (0.0, 0.0, 0.0)),
        ("vy_2p5_p", 8.0, (0.0, 2.5, 0.0)), ("stop_vy_2p5_p", 2.0, (0.0, 0.0, 0.0)),
        ("vy_2p5_n", 8.0, (0.0, -2.5, 0.0)), ("stop_vy_2p5_n", 2.0, (0.0, 0.0, 0.0)),
        ("wz_0p5_p", 8.0, (0.0, 0.0, 0.5)), ("stop_wz_0p5_p", 3.0, (0.0, 0.0, 0.0)),
        ("wz_0p5_n", 8.0, (0.0, 0.0, -0.5)), ("stop_wz_0p5_n", 3.0, (0.0, 0.0, 0.0)),
        ("wz_1p0_p", 8.0, (0.0, 0.0, 1.0)), ("stop_wz_1p0_p", 3.0, (0.0, 0.0, 0.0)),
        ("wz_1p0_n", 8.0, (0.0, 0.0, -1.0)), ("stop_wz_1p0_n", 3.0, (0.0, 0.0, 0.0)),
        ("wz_1p5_p", 8.0, (0.0, 0.0, 1.5)), ("stop_wz_1p5_p", 3.0, (0.0, 0.0, 0.0)),
        ("wz_1p5_n", 8.0, (0.0, 0.0, -1.5)), ("stop_wz_1p5_n", 3.0, (0.0, 0.0, 0.0)),
        ("wz_2p0_p", 8.0, (0.0, 0.0, 2.0)), ("stop_wz_2p0_p", 3.0, (0.0, 0.0, 0.0)),
        ("wz_2p0_n", 8.0, (0.0, 0.0, -2.0)), ("stop_wz_2p0_n", 3.0, (0.0, 0.0, 0.0)),
        ("static_b", 30.0, (0.0, 0.0, 0.0)),
    ]),
    # static + circle + static: rest threshold / fc screening
    "opt_static": (8.0, [
        ("static_a", 60.0, (0.0, 0.0, 0.0)),
        ("circle", 25.0, (0.3, 0.0, 0.5)),
        ("static_b", 60.0, (0.0, 0.0, 0.0)),
    ]),
    # in-place yaw both directions at 0.5..2.0: k / rot / odom screening
    "opt_yaw": (8.0, [
        ("static_a", 10.0, (0.0, 0.0, 0.0)),
        ("wz_0p5_p", 8.0, (0.0, 0.0, 0.5)), ("stop_wz_0p5_p", 5.0, (0.0, 0.0, 0.0)),
        ("wz_0p5_n", 8.0, (0.0, 0.0, -0.5)), ("stop_wz_0p5_n", 5.0, (0.0, 0.0, 0.0)),
        ("wz_1p0_p", 8.0, (0.0, 0.0, 1.0)), ("stop_wz_1p0_p", 5.0, (0.0, 0.0, 0.0)),
        ("wz_1p0_n", 8.0, (0.0, 0.0, -1.0)), ("stop_wz_1p0_n", 5.0, (0.0, 0.0, 0.0)),
        ("wz_1p5_p", 8.0, (0.0, 0.0, 1.5)), ("stop_wz_1p5_p", 5.0, (0.0, 0.0, 0.0)),
        ("wz_1p5_n", 8.0, (0.0, 0.0, -1.5)), ("stop_wz_1p5_n", 5.0, (0.0, 0.0, 0.0)),
        ("wz_2p0_p", 8.0, (0.0, 0.0, 2.0)), ("stop_wz_2p0_p", 5.0, (0.0, 0.0, 0.0)),
        ("wz_2p0_n", 8.0, (0.0, 0.0, -2.0)), ("stop_wz_2p0_n", 5.0, (0.0, 0.0, 0.0)),
    ]),
}


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(object):
    def __init__(self, args):
        self.args = args
        self.lock = None
        # latest messages
        self.odom = None
        self.odom_stamp = 0.0
        self.joint = None
        self.joint_stamp = 0.0
        self.gt = None
        self.gt_stamp = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.seg = ""
        self.pid = None
        self.pid_stamp = 0.0
        self.counts = {"odom": 0, "joint": 0, "gt": 0}
        self.last_counts = dict(self.counts)
        self.hero_idx = None
        self.notes = []

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_stamp = msg.header.stamp.to_sec()
        self.counts["odom"] += 1

    def joint_cb(self, msg):
        self.joint = dict(zip(msg.name, msg.velocity))
        self.joint_stamp = msg.header.stamp.to_sec()
        self.counts["joint"] += 1

    def pid_cb(self, msg):
        self.pid = msg
        self.pid_stamp = msg.header.stamp.to_sec()

    def model_cb(self, msg):
        if self.hero_idx is None:
            try:
                self.hero_idx = list(msg.name).index("hero")
            except ValueError:
                return
        if self.hero_idx < len(msg.pose):
            self.gt = msg
            self.gt_stamp = rospy.Time.now().to_sec()
            self.counts["gt"] += 1


def wait_for(pred, timeout, what):
    t0 = time.time()
    while not pred():
        if rospy.is_shutdown():
            raise RuntimeError("shutdown while waiting for " + what)
        if time.time() - t0 > timeout:
            raise RuntimeError("timeout waiting for " + what)
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="main", choices=sorted(SCENARIOS))
    ap.add_argument("--out", required=True, help="output prefix (csv/json)")
    ap.add_argument("--rate", type=float, default=200.0)
    ap.add_argument("--no-unpause", action="store_true")
    ap.add_argument("--pid", action="store_true",
                    help="also record left_front wheel PidState")
    ap.add_argument("--start-delay", type=float, default=2.0,
                    help="seconds to keep recording before unpausing")
    args = ap.parse_args()
    if not os.path.isdir(os.path.dirname(os.path.abspath(args.out))):
        os.makedirs(os.path.dirname(os.path.abspath(args.out)))

    rospy.init_node("hero_test_recorder", anonymous=True)
    rec = Recorder(args)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/odom", Odometry, rec.odom_cb, queue_size=50)
    rospy.Subscriber("/joint_states", JointState, rec.joint_cb, queue_size=50)
    rospy.Subscriber("/gazebo/model_states", ModelStates, rec.model_cb, queue_size=10)
    if args.pid:
        rospy.Subscriber("/controller/hero_chassis_controller/left_front_wheel_pid/state",
                         PidState, rec.pid_cb, queue_size=20)

    wait_for(lambda: cmd_pub.get_num_connections() > 0, 60.0, "/cmd_vel subscriber")

    csv_f = open(args.out + ".csv", "w", newline="")
    writer = csv.writer(csv_f)
    writer.writerow(COLUMNS)
    csv_f.flush()

    markers = {"scenario": args.scenario, "start_wall": time.time(),
               "rate": args.rate, "segments": []}

    def log_row():
        sim_t = rospy.Time.now().to_sec()
        wall_t = time.time()
        row = [wall_t, sim_t]
        o = rec.odom
        if o is not None:
            p = o.pose.pose.position
            yaw = quat_to_yaw(o.pose.pose.orientation)
            tw = o.twist.twist
            row += [rec.odom_stamp, sim_t - rec.odom_stamp,
                    rec.counts["odom"] - rec.last_counts["odom"],
                    rec.counts["joint"] - rec.last_counts["joint"],
                    rec.counts["gt"] - rec.last_counts["gt"],
                    rec.gt.pose[rec.hero_idx].position.x if rec.gt else float("nan"),
                    rec.gt.pose[rec.hero_idx].position.y if rec.gt else float("nan"),
                    rec.gt.pose[rec.hero_idx].position.z if rec.gt else float("nan"),
                    quat_to_yaw(rec.gt.pose[rec.hero_idx].orientation) if rec.gt else float("nan"),
                    rec.gt.twist[rec.hero_idx].linear.x if rec.gt else float("nan"),
                    rec.gt.twist[rec.hero_idx].linear.y if rec.gt else float("nan"),
                    rec.gt.twist[rec.hero_idx].linear.z if rec.gt else float("nan"),
                    rec.gt.twist[rec.hero_idx].angular.z if rec.gt else float("nan"),
                    p.x, p.y, yaw, tw.linear.x, tw.linear.y, tw.angular.z]
        else:
            row += [float("nan")] * 19
        if rec.joint is not None:
            row += [rec.joint.get(n, float("nan")) for n in WHEEL_NAMES]
        else:
            row += [float("nan")] * 4
        row += list(rec.cmd)
        if rec.pid is not None:
            row += [rec.pid_stamp, rec.pid.error, rec.pid.p_term,
                    rec.pid.i_term, rec.pid.d_term, rec.pid.output]
        else:
            row += [float("nan")] * 6
        row += [rec.seg]
        writer.writerow(row)
        rec.last_counts = dict(rec.counts)

    # The world is started paused: /clock does not move and any rospy call that
    # consults ROS time blocks.  So do the unpause here, then use sim time only.
    rospy.loginfo("waiting %.1f s (wall) before unpausing", args.start_delay)
    time.sleep(args.start_delay)
    if not args.no_unpause:
        try:
            rospy.wait_for_service("/gazebo/unpause_physics", timeout=20.0)
            rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
            rospy.loginfo("unpaused physics")
        except Exception as exc:  # noqa: BLE001
            rospy.logerr("could not unpause: %s", exc)
    t0 = rospy.Time.now().to_sec()
    markers["unpause_sim"] = t0
    markers["unpause_wall"] = time.time()

    rate = rospy.Rate(args.rate)
    settle = SCENARIOS[args.scenario][0]
    t_settle = t0 + settle
    while rospy.Time.now().to_sec() < t_settle and not rospy.is_shutdown():
        log_row()
        rate.sleep()
    markers["segments"].append({"label": "settle", "t0": t0,
                                "t1": rospy.Time.now().to_sec(), "cmd": [0, 0, 0]})
    rospy.loginfo("settle done at sim %.3f, starting segments", rospy.Time.now().to_sec())

    for label, duration, cmd in SCENARIOS[args.scenario][1]:
        rec.seg = label
        rec.cmd = cmd
        publish_once = label.startswith("stale")
        twist = Twist(linear=_v3(cmd[0], cmd[1], 0.0),
                      angular=_v3(0.0, 0.0, cmd[2]))
        if publish_once:
            cmd_pub.publish(twist)
        else:
            cmd_pub.publish(twist)
        t0 = rospy.Time.now().to_sec()
        t1 = t0 + duration
        while rospy.Time.now().to_sec() < t1 and not rospy.is_shutdown():
            if not publish_once:
                cmd_pub.publish(twist)
            log_row()
            rate.sleep()
        markers["segments"].append({"label": label, "t0": t0,
                                    "t1": rospy.Time.now().to_sec(),
                                    "cmd": list(cmd)})
        rospy.loginfo("%s done at sim %.3f", label, rospy.Time.now().to_sec())

    rec.seg = "end"
    rec.cmd = (0.0, 0.0, 0.0)
    zero = Twist()
    for _ in range(20):
        cmd_pub.publish(zero)
        log_row()
        rate.sleep()

    csv_f.flush()
    csv_f.close()
    markers["end_wall"] = time.time()
    markers["counts"] = dict(rec.counts)
    with open(args.out + ".json", "w") as f:
        json.dump(markers, f, indent=2)
    rospy.loginfo("wrote %s.csv/.json, counts=%s", args.out, rec.counts)
    return 0


def _v3(x, y, z):
    from geometry_msgs.msg import Vector3
    return Vector3(x, y, z)


if __name__ == "__main__":
    sys.exit(main())
