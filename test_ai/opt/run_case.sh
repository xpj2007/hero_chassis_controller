#!/bin/bash
# Run one optimization case: copy of controllers.yaml with overrides + scenario.
#   run_case.sh CASE SCENARIO [key=value ...]
# Example:
#   run_case.sh base opt_cal
#   run_case.sh k394 opt_cal k_effective=0.394
#   run_case.sh rest08 opt_static pid_rest_enter_threshold=0.8
WS=/home/ros/catkin_ws
T=$WS/src/hero_chassis_controller/test_ai
CASE=${1:?case name}
SCEN=${2:?scenario}
shift 2
mkdir -p "$T/opt/params" "$T/opt/results"
PARAM="$T/opt/params/$CASE.yaml"
python3 "$T/make_params.py" "$PARAM" "$@" || exit 2
echo "=== [case $CASE] scenario=$SCEN params=$PARAM"
"$T/run_session.sh" "$CASE" "$SCEN" "$PARAM" || exit 3
python3 "$T/opt/opt_metrics.py" "$T/data/$CASE.csv" "$T/opt/results/$CASE.json" || exit 4
