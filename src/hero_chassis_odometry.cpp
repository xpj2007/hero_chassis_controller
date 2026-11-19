#include "hero_chassis_controller/hero_chassis_odometry.h"
#include <cmath>

namespace hero_chassis_controller {

    void HeroChassisOdometry::updatePose(const ChassisVelocity& chassis_velocity,double dt) {
        const double yaw_mid = chassis_pose_.yaw + 0.5 * chassis_velocity.wz * dt;
        const double c = std::cos(yaw_mid);
        const double s = std::sin(yaw_mid);
        chassis_pose_.x += (chassis_velocity.vx * c - chassis_velocity.vy * s )* dt;
        chassis_pose_.y += (chassis_velocity.vx * s + chassis_velocity.vy * c )* dt;
        chassis_pose_.yaw += chassis_velocity.wz * dt;
        if (chassis_pose_.yaw > M_PI || chassis_pose_.yaw < -M_PI) {
            chassis_pose_.yaw = std::atan2(std::sin(chassis_pose_.yaw), std::cos(chassis_pose_.yaw));
        }
    }

    const ChassisPose& HeroChassisOdometry::getChassisPose() const {
        return chassis_pose_;
    }

    void HeroChassisOdometry::resetPose() {
        chassis_pose_.x = 0.0;
        chassis_pose_.y = 0.0;
        chassis_pose_.yaw = 0.0;
    }

} // namespace hero_chassis_controller