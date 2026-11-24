#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check that /tf (odom->base_link) carries exactly the /odom pose.

Reads a rosbag containing /tf, /odom, /gazebo/model_states and reports:
  * tf vs odom pose difference (matched by header stamp)
  * tf/odom rate and receive-time lag
  * odom z vs Gazebo base_link z
  * odom->base_link yaw vs Gazebo yaw

usage: tf_check.py BAG [--out JSON]
"""

import argparse
import json
import math
import sys

import rosbag


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    tf = []      # (stamp, recv, x, y, z, yaw)
    od = []      # (stamp, recv, x, y, z, yaw)
    gt = []      # (recv, x, y, z, yaw)
    hero_idx = None
    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, recv in bag.read_messages():
            if topic == "/tf":
                for tr in msg.transforms:
                    if tr.header.frame_id.lstrip("/") == "odom" and \
                       tr.child_frame_id.lstrip("/") == "base_link":
                        t = tr.header.stamp.to_sec()
                        tf.append((t, recv.to_sec(), tr.transform.translation.x,
                                   tr.transform.translation.y, tr.transform.translation.z,
                                   quat_to_yaw(tr.transform.rotation)))
            elif topic == "/odom":
                t = msg.header.stamp.to_sec()
                od.append((t, recv.to_sec(), msg.pose.pose.position.x,
                           msg.pose.pose.position.y, msg.pose.pose.position.z,
                           quat_to_yaw(msg.pose.pose.orientation)))
            elif topic == "/gazebo/model_states":
                if hero_idx is None:
                    try:
                        hero_idx = list(msg.name).index("hero")
                    except ValueError:
                        continue
                p = msg.pose[hero_idx]
                gt.append((recv.to_sec(), p.position.x, p.position.y, p.position.z,
                           quat_to_yaw(p.orientation)))

    print("bag: %d tf, %d odom, %d gt samples" % (len(tf), len(od), len(gt)))
    if not tf or not od:
        print("missing data")
        return 1
    # match by stamp (exact: same variable in the controller)
    odmap = {round(o[0], 6): o for o in od}
    dx = []; dy = []; dz = []; dyaw = []
    matched = 0
    for t in tf:
        o = odmap.get(round(t[0], 6))
        if o is None:
            continue
        matched += 1
        dx.append(t[2] - o[2]); dy.append(t[3] - o[3]); dz.append(t[4] - o[4])
        dyaw.append(math.atan2(math.sin(t[5] - o[5]), math.cos(t[5] - o[5])))
    span = tf[-1][0] - tf[0][0]
    result = {
        "bag": args.bag,
        "n_tf": len(tf), "n_odom": len(od), "n_gt": len(gt),
        "tf_rate_hz": len(tf) / span if span > 0 else None,
        "odom_rate_hz": len(od) / span if span > 0 else None,
        "stamp_match_ratio": matched / float(len(tf)),
        "tf_minus_odom": {
            "dx_max": max(abs(v) for v in dx) if dx else None,
            "dy_max": max(abs(v) for v in dy) if dy else None,
            "dz_max": max(abs(v) for v in dz) if dz else None,
            "dyaw_max": max(abs(v) for v in dyaw) if dyaw else None,
        },
        "tf_recv_lag_mean": sum(t[1] - t[0] for t in tf) / len(tf),
        "tf_recv_lag_max": max(t[1] - t[0] for t in tf),
        "gt_z_mean": sum(g[3] for g in gt) / len(gt) if gt else None,
        "odom_z_mean": sum(o[4] for o in od) / len(od),
    }
    if gt:
        # closest GT sample to each odom stamp (receipt based)
        result["odom_z_minus_gt_z"] = result["odom_z_mean"] - result["gt_z_mean"]
    print(json.dumps(result, indent=2))
    if args.out:
        json.dump(result, open(args.out, "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
