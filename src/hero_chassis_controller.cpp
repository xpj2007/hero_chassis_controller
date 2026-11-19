#include "hero_chassis_controller/hero_chassis_controller.h"

#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hero_chassis_controller
{
    bool ChassisControllerConfig::loadParams(ros::NodeHandle& controller_nh)
    {
        controller_nh.param("wheel_radius", wheel_radius, 0.0);
        controller_nh.param("wheel_base", wheel_base, 0.0);
        controller_nh.param("wheel_track", wheel_track, 0.0);
        controller_nh.param("max_effort", max_effort, 5.0);
        controller_nh.param("mode", mode, 0);

        controller_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
        controller_nh.param<std::string>("odometry_topic", odometry_topic, "/odom");
        controller_nh.param<std::string>("odom_frame", odom_frame, "odom");
        controller_nh.param<std::string>("base_frame", base_frame, "base_link");

        return true;
    }

    bool HeroChassisWheelPid::loadPid(ros::NodeHandle& controller_nh)
    {
        lf_.init(ros::NodeHandle(controller_nh, "left_front_wheel_pid"));
        rf_.init(ros::NodeHandle(controller_nh, "right_front_wheel_pid"));
        lb_.init(ros::NodeHandle(controller_nh, "left_back_wheel_pid"));
        rb_.init(ros::NodeHandle(controller_nh, "right_back_wheel_pid"));

        return true;
    }

    bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        // 加载配置参数
        config_.loadParams(controller_nh);

        // 关节注册
        left_front_wheel_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        right_front_wheel_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        left_back_wheel_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        right_back_wheel_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        // 加载pid参数
        wheel_pid_.loadPid(controller_nh);

        // 初始化运动学和里程计
        kinematics_.init(config_.wheel_radius, config_.wheel_base, config_.wheel_track);
        odometry_.resetPose();

        // 订阅速度命令
        cmd_vel_subscriber_ = controller_nh.subscribe(config_.cmd_vel_topic, 1, &HeroChassisController::doCmdVelCallback, this);

        // 初始化发布里程计
        odometry_publisher_ = controller_nh.advertise<nav_msgs::Odometry>(config_.odometry_topic, 1);

        return true;
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
    {
        // 获取当前轮速
        WheelVelocity wheel_velocity_current{
            left_front_wheel_joint_.getVelocity(),
            left_back_wheel_joint_.getVelocity(),
            right_front_wheel_joint_.getVelocity(),
            right_back_wheel_joint_.getVelocity()
        };
        // 计算当前底盘速度
        const ChassisVelocity chassis_velocity_current= kinematics_.forwardKinematics(wheel_velocity_current);


        // 里程计更新和发布
        odometry_.updatePose(chassis_velocity_current, period.toSec());
        // 发布里程计消息(函数未实现)

        // 计算轮速目标值
        WheelVelocity wheel_velocity_target = kinematics_.inverseKinematics(chassis_velocity_target_);

        // PID控制器计算输出
        wheel_pid_.update(wheel_velocity_target, wheel_velocity_current, period);

        // 设置关节输出
        left_front_wheel_joint_.setCommand(wheel_pid_.lf_output_);
        right_front_wheel_joint_.setCommand(wheel_pid_.rf_output_);
        left_back_wheel_joint_.setCommand(wheel_pid_.lb_output_);
        right_back_wheel_joint_.setCommand(wheel_pid_.rb_output_);
        
    }

    void HeroChassisController::doCmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg)
    {
        chassis_velocity_target_.vx = cmd_vel_msg->linear.x;
        chassis_velocity_target_.vy = cmd_vel_msg->linear.y;
        chassis_velocity_target_.wz = cmd_vel_msg->angular.z;
    }
}

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
