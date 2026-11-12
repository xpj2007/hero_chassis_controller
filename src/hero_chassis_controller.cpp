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

        controller_nh.getParam("wheel_radius",wheel_radius);
        double wheel_track;
        controller_nh.getParam("wheel_track",wheel_track);
        double wheel_base;
        controller_nh.getParam("wheel_base",wheel_base);
        K_Lx_Ly=wheel_track/2+wheel_base/2;

        if (!left_front_pid_.initParam(controller_nh.resolveName("left_front_pid")))
            return false;
        if (!right_front_pid_.initParam(controller_nh.resolveName("right_front_pid")))
            return false;
        if (!left_back_pid_.initParam(controller_nh.resolveName("left_back_pid")))
            return false;
        if (!right_back_pid_.initParam(controller_nh.resolveName("right_back_pid")))
            return false;

        cmd_vel_sub_ = root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::doCmdVelCallback, this);

        return true;
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
    {

        left_front_target_V=(target_Vx-target_Vy-target_Az*K_Lx_Ly)/wheel_radius;
        left_back_target_V=(target_Vx+target_Vy-target_Az*K_Lx_Ly)/wheel_radius;
        right_front_target_V=(target_Vx+target_Vy+target_Az*K_Lx_Ly)/wheel_radius;
        right_back_target_V=(target_Vx-target_Vy+target_Az*K_Lx_Ly)/wheel_radius;

        double out_effort[4]={};
        out_effort[0]=left_front_pid_.computeCommand(left_front_target_V-left_front_joint_.getVelocity(),period);
        out_effort[1]=left_back_pid_.computeCommand(left_back_target_V-left_back_joint_.getVelocity(),period);
        out_effort[2]=right_front_pid_.computeCommand(right_front_target_V-right_front_joint_.getVelocity(),period);
        out_effort[3]=right_back_pid_.computeCommand(right_back_target_V-right_back_joint_.getVelocity(),period);

        //阶越测试
        // if ((time - last_change_).toSec() > 3) {
        //     V+=4;
        //     V%=8;
        //     last_change_ = time;
        // }
        // out_effort[0]=left_front_pid_.computeCommand(V-left_front_joint_.getVelocity(),period);
        // out_effort[1]=left_back_pid_.computeCommand(V-left_back_joint_.getVelocity(),period);
        // out_effort[2]=right_front_pid_.computeCommand(V-right_front_joint_.getVelocity(),period);
        // out_effort[3]=right_back_pid_.computeCommand(V-right_back_joint_.getVelocity(),period);

        for(int i=0;i<4;i++)
        {
            if(out_effort[i]>5)out_effort[i]=5;
            else if(out_effort[i]<-5)out_effort[i]=-5;
        }
        left_front_joint_.setCommand(out_effort[0]);
        left_back_joint_.setCommand(out_effort[1]);
        right_front_joint_.setCommand(out_effort[2]);
        right_back_joint_.setCommand(out_effort[3]);
    }

    void HeroChassisController::doCmdVelCallback(const geometry_msgs::Twist& msg)
    {
        this->target_Vx=msg.linear.x;
        this->target_Vy=msg.linear.y;
        this->target_Az=msg.angular.z;
    }


}

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
