#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>


namespace hero_chassis_controller
{
    class HeroChassisController :
    public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
        bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

        void update(const ros::Time& time, const ros::Duration& period) override;

        void doCmdVelCallback(const geometry_msgs::Twist& msg);

        private:

        ros::Subscriber cmd_vel_sub_;

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

        double target_Vx=0;
        double target_Vy=0;
        double target_Az=0;

        double K_Lx_Ly=0;
        double wheel_radius=0;




    };


}



#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
