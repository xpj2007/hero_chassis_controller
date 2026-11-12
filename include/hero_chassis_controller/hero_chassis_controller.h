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


namespace hero_chassis_controller
{
    class HeroChassisController :
    public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
        HeroChassisController():tf_listener_(tf_buffer_){}
        bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

        void update(const ros::Time& time, const ros::Duration& period) override;

        private:
        ros::Subscriber cmd_vel_sub_;
        ros::Publisher odom_pub_;
        tf2_ros::TransformBroadcaster odom_broadcaster_;
        tf2_ros::Buffer tf_buffer_{ros::Duration(1.0)};
        tf2_ros::TransformListener tf_listener_;

        hardware_interface::JointHandle left_front_joint_;
        hardware_interface::JointHandle left_back_joint_;
        hardware_interface::JointHandle right_front_joint_;
        hardware_interface::JointHandle right_back_joint_;

        control_toolbox::Pid left_front_pid_;
        control_toolbox::Pid left_back_pid_;
        control_toolbox::Pid right_front_pid_;
        control_toolbox::Pid right_back_pid_;

        double left_front_target_V=0;
        double right_back_target_V=0;
        double left_back_target_V=0;
        double right_front_target_V=0;

        //调pid时用的测试参数
        // ros::Time last_change_;
        // int V=0;

        double target_Vx_odom_=0;
        double target_Vy_odom_=0;
        double target_Az_odom_=0;

        double target_Vx=0;
        double target_Vy=0;
        double target_Az=0;

        double current_Vx=0;
        double current_Vy=0;
        double current_Az=0;

        double K_Lx_Ly=0;
        double wheel_radius=0;

        void publishOdometry(const ros::Time& time,const ros::Duration& period);
        void odomToBaselinkSpeed(const ros::Time& time,const ros::Duration& period);
        void doCmdVelCallback(const geometry_msgs::Twist& msg);

        double X=0;
        double Y=0;
        double YAW=0;
        int mode=0;



    };


}



#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
