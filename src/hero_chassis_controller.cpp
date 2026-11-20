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
        controller_nh.param("effort_limit", effort_limit, 5.0);
        controller_nh.param("mode", mode, 0);

        controller_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
        controller_nh.param<std::string>("odometry_topic", odometry_topic, "/odom");
        controller_nh.param<std::string>("odom_frame", odom_frame, "odom");
        controller_nh.param<std::string>("base_frame", base_frame, "base_link");

        controller_nh.param("chassis_line_accel_limit", chassis_line_accel_limit, 0.5);
        controller_nh.param("chassis_angular_accel_limit", chassis_angular_accel_limit, 1.5);
        controller_nh.param("zero_speed_threshold", zero_speed_threshold, 0.2);

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

        // 里程计坐标更新
        odometry_.updatePose(chassis_velocity_current, period.toSec());

        // tf坐标变换广播
        broadcastTransform(odometry_.getChassisPose(), time);

        // 里程计信息发布
        publishOdometry(odometry_.getChassisPose(),chassis_velocity_current,time);

        // 目标速度的加速度限幅
        chassis_velocity_applied_=limitRate(chassis_velocity_target_, chassis_velocity_applied_, config_.chassis_line_accel_limit, config_.chassis_angular_accel_limit, period.toSec());
        // 计算轮速目标值
        WheelVelocity wheel_velocity_target = kinematics_.inverseKinematics(chassis_velocity_applied_);

        // PID控制器计算输出
        wheel_pid_.update(wheel_velocity_target, wheel_velocity_current, period, config_.effort_limit);

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

    void HeroChassisController::broadcastTransform(const ChassisPose& odom_pose, const ros::Time& time)
    {
        //欧拉角转四元数
        tf2::Quaternion q;
        q.setRPY(0,0,odom_pose.yaw);

        //tf坐标关系
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = time;
        odom_trans.header.frame_id = config_.odom_frame;
        odom_trans.child_frame_id = config_.base_frame;
        odom_trans.transform.translation.x = odom_pose.x;
        odom_trans.transform.translation.y = odom_pose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation.w = q.getW();
        odom_trans.transform.rotation.x = q.getX();
        odom_trans.transform.rotation.y = q.getY();
        odom_trans.transform.rotation.z = q.getZ();

        //发布坐标关系
        tf_broadcaster_.sendTransform(odom_trans);
        
    }

    void HeroChassisController::publishOdometry(const ChassisPose& odom_pose, const ChassisVelocity& v, const ros::Time& time){
        
        //欧拉角转四元数
        tf2::Quaternion q;
        q.setRPY(0,0,odom_pose.yaw);

        //里程计消息
        nav_msgs::Odometry odometry_msgs;
        odometry_msgs.header.stamp = time;
        odometry_msgs.header.frame_id = config_.odom_frame;
        odometry_msgs.child_frame_id = config_.base_frame;
        odometry_msgs.pose.pose.position.x = odom_pose.x;
        odometry_msgs.pose.pose.position.y = odom_pose.y;
        odometry_msgs.pose.pose.position.z = 0.0;
        odometry_msgs.pose.pose.orientation.w = q.getW();
        odometry_msgs.pose.pose.orientation.x = q.getX();
        odometry_msgs.pose.pose.orientation.y = q.getY();
        odometry_msgs.pose.pose.orientation.z = q.getZ();
        odometry_msgs.twist.twist.linear.x = v.vx;
        odometry_msgs.twist.twist.linear.y = v.vy;
        odometry_msgs.twist.twist.linear.z = 0;
        odometry_msgs.twist.twist.angular.x = 0.0;
        odometry_msgs.twist.twist.angular.y = 0.0;
        odometry_msgs.twist.twist.angular.z = v.wz;

        //发布里程计消息
        odometry_publisher_.publish(odometry_msgs);


    }

    inline double approach(double c, double t, double step){
        const double d = t - c;
        if (std::fabs(d) <= step) return t;
        return c + (d > 0.0 ? step : -step);
    }

    ChassisVelocity HeroChassisController::limitRate(const ChassisVelocity& target, const ChassisVelocity& cur, double dv_limit, double dw_limit, double dt){
        const double dv_max = dv_limit * dt;
        const double dw_max = dw_limit * dt;
        ChassisVelocity limited_result_;

        limited_result_.vx = approach(cur.vx, target.vx, dv_max);
        limited_result_.vy = approach(cur.vy, target.vy, dv_max);
        limited_result_.wz = approach(cur.wz, target.wz, dw_max);
        return limited_result_;
    }
}

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
