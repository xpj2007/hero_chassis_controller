#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_KINEMATICS_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_KINEMATICS_H

#include "hero_chassis_controller/hero_chassis_type.h"
namespace hero_chassis_controller {

class HeroChassisKinematics {
public:
    void init(double wheel_radius, double wheel_base, double wheel_track);
    ChassisVelocity forwardKinematics(const WheelVelocity& wheel_velocities_) const;
    WheelVelocity inverseKinematics(const ChassisVelocity& chassis_velocity_) const;
private:
    double wheel_radius_=0.0;
    double k_=0.0;
};


} // namespace hero_chassis_controller

#endif // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_KINEMATICS_H
