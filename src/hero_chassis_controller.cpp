#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{
    bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
    {
        left_front_joint_=effort_joint_interface->getHandle("left_front_wheel_joint");
        left_back_joint_=effort_joint_interface->getHandle("left_back_wheel_joint");
        right_front_joint_=effort_joint_interface->getHandle("right_front_wheel_joint");
        right_back_joint_=effort_joint_interface->getHandle("right_back_wheel_joint");
        return true;
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
    {
        left_back_joint_.setCommand(0);
        left_front_joint_.setCommand(0);
        right_back_joint_.setCommand(0);
        right_front_joint_.setCommand(0);
    }



}

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
