#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>


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
        hardware_interface::JointHandle right_front_joint_;
        hardware_interface::JointHandle left_back_joint_;
        hardware_interface::JointHandle right_back_joint_;
    };


}



#endif //HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
