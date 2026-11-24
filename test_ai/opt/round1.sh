#!/bin/bash
# Round 1 batch: baseline rest/yaw + first candidates. One Gazebo at a time.
T=/home/ros/catkin_ws/src/hero_chassis_controller/test_ai
$T/opt/run_case.sh stat_base opt_static || echo "FAIL stat_base"
$T/opt/run_case.sh yaw_base opt_yaw || echo "FAIL yaw_base"
$T/opt/run_case.sh yaw_k3945 opt_yaw k_effective=0.3945 || echo "FAIL yaw_k3945"
$T/opt/run_case.sh stat_odom05 opt_static odom_rest_enter_threshold=0.5 odom_rest_exit_threshold=1.0 || echo "FAIL stat_odom05"
$T/opt/run_case.sh stat_fc5 opt_static low_pass_filter_fc=5.0 || echo "FAIL stat_fc5"
echo "ROUND1_DONE"
