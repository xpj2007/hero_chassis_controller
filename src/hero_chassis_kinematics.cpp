#include "hero_chassis_controller/hero_chassis_kinematics.h"

namespace hero_chassis_controller {
    HeroChassisKinematics::HeroChassisKinematics(double wheel_radius, double wheel_base, double wheel_track)
        : wheel_radius_(wheel_radius), wheel_base_(wheel_base), wheel_track_(wheel_track) {
        k_ly_lx_ = (wheel_base_ + wheel_track_) / 2.0;
    }
    ChassisVelocity HeroChassisKinematics::ForwardKinematics(const WheelVelocity& wheel_velocities_) const {
        ChassisVelocity chassis_velocity;
        chassis_velocity.vx = (wheel_velocities_.lf + wheel_velocities_.lb + wheel_velocities_.rf + wheel_velocities_.rb) * (wheel_radius_ / 4.0);
        chassis_velocity.vy = (-wheel_velocities_.lf + wheel_velocities_.lb + wheel_velocities_.rf - wheel_velocities_.rb) * (wheel_radius_ / 4.0);
        chassis_velocity.wz = (-wheel_velocities_.lf + wheel_velocities_.lb - wheel_velocities_.rf + wheel_velocities_.rb) * (wheel_radius_ / (4.0 * k_ly_lx_));
        return chassis_velocity;
    }
    WheelVelocity HeroChassisKinematics::InverseKinematics(const ChassisVelocity& chassis_velocity_) const {
        WheelVelocity wheel_velocities;
        wheel_velocities.lf = (chassis_velocity_.vx - chassis_velocity_.vy - k_ly_lx_ * chassis_velocity_.wz) / wheel_radius_;
        wheel_velocities.lb = (chassis_velocity_.vx + chassis_velocity_.vy - k_ly_lx_ * chassis_velocity_.wz) / wheel_radius_;
        wheel_velocities.rf = (chassis_velocity_.vx + chassis_velocity_.vy + k_ly_lx_ * chassis_velocity_.wz) / wheel_radius_;
        wheel_velocities.rb = (chassis_velocity_.vx - chassis_velocity_.vy + k_ly_lx_ * chassis_velocity_.wz) / wheel_radius_;
        return wheel_velocities;
    }
}

