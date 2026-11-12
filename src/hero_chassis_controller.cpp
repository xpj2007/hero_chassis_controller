#include "hero_chassis_controller/hero_chassis_controller.h"

#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
        controller_nh.getParam("mode",mode);

        if (!left_front_pid_.initParam(controller_nh.resolveName("left_front_pid")))
            return false;
        if (!right_front_pid_.initParam(controller_nh.resolveName("right_front_pid")))
            return false;
        if (!left_back_pid_.initParam(controller_nh.resolveName("left_back_pid")))
            return false;
        if (!right_back_pid_.initParam(controller_nh.resolveName("right_back_pid")))
            return false;

        cmd_vel_sub_ = root_nh.subscribe("/cmd_vel", 1, &HeroChassisController::doCmdVelCallback, this);
        odom_pub_= root_nh.advertise<nav_msgs::Odometry>("/odom", 10);

        return true;
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
    {
        const double w_lf=left_front_joint_.getVelocity();
        const double w_lb=left_back_joint_.getVelocity();
        const double w_rf=right_front_joint_.getVelocity();
        const double w_rb=right_back_joint_.getVelocity();

        current_Vx = wheel_radius / 4.0 * ( w_lf + w_rf + w_lb + w_rb);
        current_Vy = wheel_radius / 4.0 * (-w_lf + w_rf + w_lb - w_rb);
        current_Az = wheel_radius / (4.0 * K_Lx_Ly) * (-w_lf + w_rf - w_lb + w_rb);

        publishOdometry(time,period);

        odomToBaselinkSpeed(time,period);
        left_front_target_V=(target_Vx-target_Vy-target_Az*K_Lx_Ly)/wheel_radius;
        left_back_target_V=(target_Vx+target_Vy-target_Az*K_Lx_Ly)/wheel_radius;
        right_front_target_V=(target_Vx+target_Vy+target_Az*K_Lx_Ly)/wheel_radius;
        right_back_target_V=(target_Vx-target_Vy+target_Az*K_Lx_Ly)/wheel_radius;

        double out_effort[4]={};
        out_effort[0]=left_front_pid_.computeCommand(left_front_target_V-w_lf,period);
        out_effort[1]=left_back_pid_.computeCommand(left_back_target_V-w_lb,period);
        out_effort[2]=right_front_pid_.computeCommand(right_front_target_V-w_rf,period);
        out_effort[3]=right_back_pid_.computeCommand(right_back_target_V-w_rb,period);

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
        if (mode==0)
        {
            this->target_Vx=msg.linear.x;
            this->target_Vy=msg.linear.y;
            this->target_Az=msg.angular.z;
        }else
        {
            this->target_Vx_odom_ = msg.linear.x;
            this->target_Vy_odom_ = msg.linear.y;
            this->target_Az_odom_ = msg.angular.z;
        }
    }

    void HeroChassisController::publishOdometry(const ros::Time& time,const ros::Duration& period)
    {
        const double dt = period.toSec();
        const double delta_yaw = current_Az * dt;
        X   += (current_Vx * std::cos(YAW) - current_Vy * std::sin(YAW)) * dt;
        Y   += (current_Vx * std::sin(YAW) + current_Vy * std::cos(YAW)) * dt;
        YAW += delta_yaw;
        YAW = std::atan2(std::sin(YAW), std::cos(YAW));

        tf2::Quaternion qtn;
        qtn.setRPY(0,0,YAW);

        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp=time;
        tfs.header.frame_id="odom";
        tfs.child_frame_id="base_link";
        tfs.transform.translation.x=X;
        tfs.transform.translation.y=Y;
        tfs.transform.translation.z=0.0;
        tfs.transform.rotation.w=qtn.getW();
        tfs.transform.rotation.x=qtn.getX();
        tfs.transform.rotation.y=qtn.getY();
        tfs.transform.rotation.z=qtn.getZ();

        odom_broadcaster_.sendTransform(tfs);


        nav_msgs::Odometry odom_tfs2;
        odom_tfs2.header.stamp=time;
        odom_tfs2.header.frame_id="odom";
        odom_tfs2.child_frame_id="base_link";
        odom_tfs2.pose.pose.position.x=X;
        odom_tfs2.pose.pose.position.y=Y;
        odom_tfs2.pose.pose.position.z=0.0;
        odom_tfs2.pose.pose.orientation.w=qtn.getW();
        odom_tfs2.pose.pose.orientation.x=qtn.getX();
        odom_tfs2.pose.pose.orientation.y=qtn.getY();
        odom_tfs2.pose.pose.orientation.z=qtn.getZ();
        odom_tfs2.pose.pose.orientation.w=qtn.getW();
        odom_tfs2.twist.twist.linear.x=current_Vx;
        odom_tfs2.twist.twist.linear.y=current_Vy;
        odom_tfs2.twist.twist.linear.z=0.0;
        odom_tfs2.twist.twist.angular.x=0.0;
        odom_tfs2.twist.twist.angular.y=0.0;
        odom_tfs2.twist.twist.angular.z=current_Az;

        odom_pub_.publish(odom_tfs2);


    }

    void HeroChassisController::odomToBaselinkSpeed(const ros::Time& time,const ros::Duration& period)
    {
        if (mode==0)return ;
        geometry_msgs::TransformStamped odom_in_base;
        try
        {
            odom_in_base = tf_buffer_.lookupTransform("base_link", "odom", ros::Time(0), ros::Duration(0.0));
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
        geometry_msgs::Vector3Stamped v_in, v_out;
        v_in.header   = odom_in_base.header;
        v_in.vector.x = target_Vx_odom_;
        v_in.vector.y = target_Vy_odom_;
        v_in.vector.z = 0.0;

        tf2::doTransform(v_in, v_out, odom_in_base);

        target_Vx = v_out.vector.x;
        target_Vy = v_out.vector.y;
        target_Az = target_Az_odom_;

    }

}

    PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
