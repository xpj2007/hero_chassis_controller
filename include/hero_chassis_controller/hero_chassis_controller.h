#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>


namespace hero_chassis_controller
{
    class HeroChassisController :
    public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
        bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

        void update(const ros::Time& time, const ros::Duration& period) override;

        private:
        hardware_interface::JointHandle left_front_joint_;
        hardware_interface::JointHandle left_back_joint_;
        hardware_interface::JointHandle right_front_joint_;
        hardware_interface::JointHandle right_back_joint_;

        control_toolbox::Pid left_front_pid_;
        control_toolbox::Pid left_back_pid_;
        control_toolbox::Pid right_front_pid_;
        control_toolbox::Pid right_back_pid_;

        double left_front_target_V=2.0;
        double right_back_target_V=2.0;
        double left_back_target_V=2.0;
        double right_front_target_V=2.0;

        //调pid时用的测试参数
        // ros::Time last_change_;
        // int V=0;


    };


}



#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
