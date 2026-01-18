
#ifndef PID_CONTROLLER
#define PID_CONTROLLER

#include <PID_v1.h>
#include "Odometry.h"
#include "MotorDriver.h"

#define Kp 500
#define Ki 68
#define Kd 55

#define ALPHA_LPF 0.75  // Factor for velocity LPF

class WheelController {
public:
    WheelController(Odometry& odom, MotorDriver& motor);

    void begin();
    void setTargetVelocity(float rad_per_sec);
    void update();   // call at fixed interval

    void disablePID(); // E-stop
    void enablePID(); // E-stop

private:
    Odometry& odom_;
    MotorDriver& motor_;

    double target_vel_;
    double measured_vel_;
    double measured_vel_filtered;
    double pwm_out_;

    PID pid_;
};

#endif