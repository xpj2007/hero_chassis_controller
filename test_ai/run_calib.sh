#!/bin/bash
# Original-rule calibration session (test_ai, runs inside the ros1 container).
#   run_calib.sh [PARAMS_YAML]
# Starts the same headless robot as the tests, unpaused, then runs the original
# r (straight line) and k (in-place rotation) calibration procedures.
WS=/home/ros/catkin_ws
T=$WS/src/hero_chassis_controller/test_ai
PARAMS=${1:-$WS/src/hero_chassis_controller/config/controllers.yaml}
mkdir -p "$T/calib" "$T/logs" "$T/data"
export ROS_HOME=$T/.ros
export ROS_LOG_DIR=$ROS_HOME/log
mkdir -p "$ROS_HOME"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311
set -u

cleanup() {
  pkill -INT -f "[c]alibrate_ai.py" 2>/dev/null
  pkill -INT -f "[r]oslaunch.*test_bringup" 2>/dev/null
  sleep 2
  pkill -KILL -f "[c]alibrate_ai.py" 2>/dev/null
  pkill -KILL -f "[r]oslaunch.*test_bringup" 2>/dev/null
  for n in gzserver gzclient rosmaster rosout; do pkill -KILL -x "$n" 2>/dev/null; done
  sleep 1
}
trap cleanup EXIT

cleanup
rm -f "$T/calib/r.json" "$T/calib/k.json"
echo "=== [calib] clean, launching (params=$PARAMS)"
roslaunch "$T/test_bringup.launch" params:="$PARAMS" gui:=false paused:=false spawn_z:=0.5 \
  > "$T/logs/launch_calib.log" 2>&1 &
LPID=$!

ok=0
for _ in $(seq 1 160); do
  if ! kill -0 $LPID 2>/dev/null; then echo "roslaunch died"; tail -40 "$T/logs/launch_calib.log"; exit 1; fi
  if rosservice list 2>/dev/null | grep -q "/controller/hero_chassis_controller/left_front_wheel_pid/set_parameters"; then
    ok=1; break
  fi
  sleep 0.5
done
if [ "$ok" != 1 ]; then echo "controller did not load"; tail -50 "$T/logs/launch_calib.log"; exit 1; fi
echo "=== [calib] controller loaded; r calibration"
sleep 2

timeout --signal=INT --kill-after=10 240 \
  python3 "$T/calibrate_ai.py" --mode r --speed 0.3 --duration 10 --settle 5 \
    --tail 1 --r-old 0.07625 --out "$T/calib/r.json" \
  > "$T/logs/calib_r.log" 2>&1
RC1=$?
echo "=== [calib] r exit $RC1"; tail -25 "$T/logs/calib_r.log"

echo "=== [calib] k calibration"
timeout --signal=INT --kill-after=10 420 \
  python3 "$T/calibrate_ai.py" --mode k --speed 0.5 --duration 60 --settle 5 \
    --tail 1 --k-old 0.4 --out "$T/calib/k.json" \
  > "$T/logs/calib_k.log" 2>&1
RC2=$?
echo "=== [calib] k exit $RC2"; tail -30 "$T/logs/calib_k.log"

kill -INT $LPID 2>/dev/null
for _ in $(seq 1 30); do kill -0 $LPID 2>/dev/null || break; sleep 0.5; done
kill -KILL $LPID 2>/dev/null
sleep 1
cleanup
trap - EXIT
pkill -KILL -x gzserver 2>/dev/null

echo "=== [calib] files"
ls -l "$T/calib"/*.json 2>/dev/null
if [ $RC1 -ne 0 ] || [ $RC2 -ne 0 ]; then exit 1; fi
exit 0
