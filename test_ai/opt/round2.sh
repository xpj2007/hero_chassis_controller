#!/bin/bash
T=/home/ros/catkin_ws/src/hero_chassis_controller/test_ai
$T/opt/run_case.sh stat_base2 opt_static || echo "FAIL stat_base2"
$T/opt/run_case.sh stat_odom05b opt_static odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 || echo "FAIL stat_odom05b"
$T/opt/run_case.sh yaw_k3974 opt_yaw k_effective=0.3974 || echo "FAIL yaw_k3974"
$T/opt/run_case.sh yaw_k3960 opt_yaw k_effective=0.3960 || echo "FAIL yaw_k3960"
$T/opt/run_case.sh stat_fc10 opt_static low_pass_filter_fc=10.0 || echo "FAIL stat_fc10"
$T/opt/run_case.sh stat_pid08 opt_static pid_rest_enter_threshold=0.8 || echo "FAIL stat_pid08"
$T/opt/run_case.sh cal_r0759 opt_cal wheel_radius_effective=0.0759 || echo "FAIL cal_r0759"
echo "ROUND2_DONE"
