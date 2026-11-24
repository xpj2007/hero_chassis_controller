#!/bin/bash
T=/home/ros/catkin_ws/src/hero_chassis_controller/test_ai
P=$T/opt/params
L=$T/logs
mkdir -p $P $L
# W1: original k/r (for the original +0.5), tuned odom/fc
python3 $T/make_params.py $P/w1.yaml odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 low_pass_filter_fc=10.0
$T/run_session.sh main_w1 main $P/w1.yaml
python3 $T/analyze.py main_w1 --mode 1 --radius 0.076309 --k 0.390222 > $L/analysis_main_w1.log 2>&1
python3 $T/make_params.py $P/w1_mode0.yaml odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 low_pass_filter_fc=10.0 mode=0
$T/run_session.sh mode0_w1 mode0 $P/w1_mode0.yaml
python3 $T/analyze.py mode0_w1 --mode 0 --radius 0.076309 --k 0.390222 > $L/analysis_mode0_w1.log 2>&1
# W2/W3/W4: multi-speed k/r
python3 $T/make_params.py $P/w2.yaml k_effective=0.3974 wheel_radius_effective=0.0761 odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 low_pass_filter_fc=10.0
$T/run_session.sh main_w2 main $P/w2.yaml
python3 $T/analyze.py main_w2 --mode 1 --radius 0.0761 --k 0.3974 > $L/analysis_main_w2.log 2>&1
python3 $T/make_params.py $P/w2_mode0.yaml k_effective=0.3974 wheel_radius_effective=0.0761 odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 low_pass_filter_fc=10.0 mode=0
$T/run_session.sh mode0_w2 mode0 $P/w2_mode0.yaml
python3 $T/analyze.py mode0_w2 --mode 0 --radius 0.0761 --k 0.3974 > $L/analysis_mode0_w2.log 2>&1
echo "ROUND4_DONE"
