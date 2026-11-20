#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "hero_chassis_controller/hero_chassis_kinematics.h"
#include "hero_chassis_controller/hero_chassis_odometry.h"


namespace hero_chassis_controller
{
    class ChassisControllerConfig
    {
        public:
        double wheel_radius = 0.0;
        double wheel_base   = 0.0;
        double wheel_track  = 0.0;
        double effort_limit = 5.0;
        int    mode         = 0;

        double chassis_line_accel_limit    = 0.5;   
        double chassis_angular_accel_limit = 1.5;   
        double zero_speed_threshold        = 0.2; 

        std::string cmd_vel_topic = "/cmd_vel";
        std::string odometry_topic= "/odom";
        std::string odom_frame    = "odom";
        std::string base_frame    = "base_link";

        bool loadParams(ros::NodeHandle& controller_nh);
    };

    class HeroChassisWheelPid
    {
        public:
        control_toolbox::Pid lf_,rf_,lb_,rb_;
        bool loadPid(ros::NodeHandle& controller_nh);
        double lf_output_,rf_output_,lb_output_,rb_output_;

        inline double clampEffort(double effort, double limit) {
            return effort > limit ? limit : (effort < -limit ? -limit : effort);
        }

        void update(WheelVelocity& wheel_velocity_target, WheelVelocity& wheel_velocity_current, const ros::Duration& period, double effort_limit)
        {
            lf_output_ = clampEffort(lf_.computeCommand(wheel_velocity_target.lf - wheel_velocity_current.lf, period), effort_limit);
            rf_output_ = clampEffort(rf_.computeCommand(wheel_velocity_target.rf - wheel_velocity_current.rf, period), effort_limit);
            lb_output_ = clampEffort(lb_.computeCommand(wheel_velocity_target.lb - wheel_velocity_current.lb, period), effort_limit);
            rb_output_ = clampEffort(rb_.computeCommand(wheel_velocity_target.rb - wheel_velocity_current.rb, period), effort_limit);
            
        }
    };

    class HeroChassisController :
    public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
        HeroChassisController() : tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_) {};
        bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

        void update(const ros::Time& time, const ros::Duration& period) override;

        private:

        //硬件接口
        hardware_interface::JointHandle left_front_wheel_joint_;
        hardware_interface::JointHandle right_front_wheel_joint_;
        hardware_interface::JointHandle left_back_wheel_joint_;
        hardware_interface::JointHandle right_back_wheel_joint_;

        //控制器配置参数
        ChassisControllerConfig config_;

        //速度命令
        ChassisVelocity chassis_velocity_target_;
        //加速度限幅后的速度命令
        ChassisVelocity chassis_velocity_applied_;

        //PID控制器
        HeroChassisWheelPid wheel_pid_;

        //运动学模型
        HeroChassisKinematics kinematics_;
        //里程计计算
        HeroChassisOdometry odometry_;

        // 通信接口
        ros::Subscriber cmd_vel_subscriber_;
        ros::Publisher odometry_publisher_;

        // TF广播器
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // 回调函数
        void doCmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

        // 里程计消息发布函数
        void publishOdometry(const ChassisPose& odom_pose, const ChassisVelocity& v, const ros::Time& time);

        // TF广播函数
        void broadcastTransform(const ChassisPose& odom_pose, const ros::Time& time);

        // 速度限幅函数
        ChassisVelocity limitRate(const ChassisVelocity& target, const ChassisVelocity& cur, double dv_limit, double dw_limit, double dt); 


    };


}



#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
