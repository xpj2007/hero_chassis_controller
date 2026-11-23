#!/bin/bash
# Test-only session orchestrator (runs inside the ros1 container).
#   run_session.sh RUN_NAME SCENARIO [PARAMS_YAML] [TIMEOUT_S]
# Starts a clean headless simulation, drives the scenario, records CSV, and
# tears everything down again.
WS=/home/ros/catkin_ws
T=$WS/src/hero_chassis_controller/test_ai
RUN=${1:?run name}
SCEN=${2:-main}
PARAMS=${3:-$WS/src/hero_chassis_controller/config/controllers.yaml}
TIMEOUT=${4:-1800}

mkdir -p "$T/data" "$T/logs" "$T/params"
export ROS_HOME=$T/.ros
export ROS_LOG_DIR=$ROS_HOME/log
mkdir -p "$ROS_HOME"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311
set -u

cleanup() {
  pkill -INT -f "[r]ecord_scenarios.py" 2>/dev/null
  pkill -INT -f "[r]oslaunch.*test_bringup" 2>/dev/null
  sleep 2
  pkill -KILL -f "[r]ecord_scenarios.py" 2>/dev/null
  pkill -KILL -f "[r]oslaunch.*test_bringup" 2>/dev/null
  for n in gzserver gzclient rosmaster rosout; do pkill -KILL -x "$n" 2>/dev/null; done
  sleep 1
}
trap cleanup EXIT

cleanup
echo "=== [$RUN] clean, launching (scenario=$SCEN params=$PARAMS)"

roslaunch "$T/test_bringup.launch" params:="$PARAMS" gui:=false paused:=true spawn_z:="${SPAWN_Z:-0.5}" \
  > "$T/logs/launch_$RUN.log" 2>&1 &
LPID=$!

# Wait for controller_manager and for the controller to be loaded.  While the
# world is paused /clock is frozen, so rospy service *calls* would block; only
# use the master-side service list here.  The recorder unpauses the world.
ok=0
for _ in $(seq 1 160); do
  if ! kill -0 $LPID 2>/dev/null; then echo "roslaunch died"; tail -30 "$T/logs/launch_$RUN.log"; exit 1; fi
  if rosservice list 2>/dev/null | grep -q "/controller/hero_chassis_controller/left_front_wheel_pid/set_parameters"; then
    ok=1; break
  fi
  sleep 0.5
done
if [ "$ok" != 1 ]; then
  echo "controller did not load"; tail -50 "$T/logs/launch_$RUN.log"; exit 1
fi
echo "=== [$RUN] controller loaded, starting recorder (it will unpause)"

BAGPID=""
if [ "${RECORD_BAG:-0}" = "1" ]; then
  echo "=== [$RUN] recording ${BAG_SECS:-25}s rosbag /tf /odom /gazebo/model_states"
  rosbag record -O "$T/data/${RUN}_tf.bag" /tf /odom /gazebo/model_states \
    --duration="${BAG_SECS:-25}" > "$T/logs/bag_$RUN.log" 2>&1 &
  BAGPID=$!
fi

timeout --signal=INT --kill-after=20 "$TIMEOUT" \
  python3 "$T/record_scenarios.py" --scenario "$SCEN" --out "$T/data/$RUN" --pid \
  > "$T/logs/record_$RUN.log" 2>&1
RC=$?
echo "=== [$RUN] recorder exit $RC"
tail -5 "$T/logs/record_$RUN.log"

if [ -n "$BAGPID" ]; then
  kill -0 $BAGPID 2>/dev/null && { kill -INT $BAGPID; sleep 3; kill -KILL $BAGPID 2>/dev/null; }
fi

# Graceful shutdown of the launch.
kill -INT $LPID 2>/dev/null
for _ in $(seq 1 30); do kill -0 $LPID 2>/dev/null || break; sleep 0.5; done
kill -KILL $LPID 2>/dev/null
sleep 1
cleanup
trap - EXIT
pkill -KILL -x gzserver 2>/dev/null

if [ -f "$T/data/$RUN.csv" ]; then
  echo "=== [$RUN] rows: $(wc -l < "$T/data/$RUN.csv")"
  ls -l "$T/data/$RUN".{csv,json} 2>/dev/null
fi
exit $RC
