#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_TYPE_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_TYPE_H


namespace hero_chassis_controller {

typedef struct {   
    double vx = 0.0;       
    double vy = 0.0;       
    double wz = 0.0;       
}ChassisVelocity;

// 轮序固定为 [lf, lb, rf, rb]
typedef struct {
    double lf = 0.0;       
    double lb = 0.0;       
    double rf = 0.0;       
    double rb = 0.0;
}WheelVelocity;

typedef struct {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
}ChassisPose;

typedef struct { 
    bool latched = false; 
    double timer = 0.0; 
}RestState ;

} 



#endif // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_TYPE_H