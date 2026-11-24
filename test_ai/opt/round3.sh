#!/bin/bash
T=/home/ros/catkin_ws/src/hero_chassis_controller/test_ai
$T/opt/run_case.sh stat_odom05_fc10 opt_static odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 low_pass_filter_fc=10.0 || echo "FAIL stat_odom05_fc10"
$T/opt/run_case.sh stat_k3974 opt_static k_effective=0.3974 odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 low_pass_filter_fc=10.0 || echo "FAIL stat_k3974"
$T/opt/run_case.sh cal_r0761 opt_cal wheel_radius_effective=0.0761 k_effective=0.3974 odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 low_pass_filter_fc=10.0 || echo "FAIL cal_r0761"
$T/opt/run_case.sh stat_rot015 opt_static k_effective=0.3974 odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 low_pass_filter_fc=10.0 pid_rest_rot_enter_threshold=0.15 || echo "FAIL stat_rot015"
echo "ROUND3_DONE"
