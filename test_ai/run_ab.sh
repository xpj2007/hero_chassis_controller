#!/bin/bash
# Sequential A/B sessions (test scaffolding, runs inside the ros1 container).
#   1. PID gains zero           -> separates PID torque from plant/contact drift
#   2. zero_speed_threshold=1.0 -> tests the rest-latch hypothesis
#   3. spawn at rest height     -> separates spawn-drop settling from steady drift
#   4. mode 0 (body frame)      -> compares with the mode 1 main run
#   5. low-pass fc=20           -> filter effect (optional, last)
set -u
T=/home/ros/catkin_ws/src/hero_chassis_controller/test_ai
BASE=/home/ros/catkin_ws/src/hero_chassis_controller/config/controllers.yaml
mkdir -p "$T/logs"

echo "### A/B start $(date)"
echo "### 0/6 stale-cmd (no cmd_vel watchdog test)"
"$T/run_session.sh" stale stale "$BASE" 600
echo "### 1/6 pidzero"
RECORD_BAG=1 BAG_SECS=25 "$T/run_session.sh" pidzero static_short "$T/params/pid_zero.yaml" 900
echo "### 2/6 thr1"
"$T/run_session.sh" thr1 static_short "$T/params/thr_1p0.yaml" 900
echo "### 3/6 spawnlow"
SPAWN_Z=0.1466 "$T/run_session.sh" spawnlow static_short "$BASE" 900
echo "### 4/6 mode0"
"$T/run_session.sh" mode0 mode0_short "$T/params/mode0.yaml" 1200
echo "### 5/6 fc20"
"$T/run_session.sh" fc20 steps "$T/params/fc_20.yaml" 1200
echo "### A/B done $(date)"
