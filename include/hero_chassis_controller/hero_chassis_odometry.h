#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_ODOMETRY_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_ODOMETRY_H

#include "hero_chassis_controller/hero_chassis_type.h"

namespace hero_chassis_controller {

    class HeroChassisOdometry {
    public:
        void updatePose(const ChassisVelocity& chassis_velocity,double dt);
        const ChassisPose& getChassisPose() const;
        void resetPose();
    private:
        ChassisPose chassis_pose_;
        
    };


} // namespace hero_chassis_controller



#endif // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_ODOMETRY_H