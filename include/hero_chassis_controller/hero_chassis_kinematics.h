#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_KINEMATICS_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_KINEMATICS_H

#include "hero_chassis_controller/hero_chassis_type.h"
namespace hero_chassis_controller {

class HeroChassisKinematics {
public:
    HeroChassisKinematics(double wheel_radius, double wheel_base, double wheel_track);
    ChassisVelocity ForwardKinematics(const WheelVelocity& wheel_velocities_) const;
    WheelVelocity InverseKinematics(const ChassisVelocity& chassis_velocity_) const;
private:
    double wheel_radius_;
    double wheel_base_;
    double wheel_track_;
    double k_ly_lx_;
};


} // namespace hero_chassis_controller

#endif // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_KINEMATICS_H>>