#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Odometry.h"
#include "MotorDriver.h"

// ================== TUNABLE GAINS ==================
// Feedback gains (PWM per m/s)
#define KP 180.0f
#define KI 80.0f
#define KD 10.0f

// Feedforward gain (PWM per m/s)
// YOU tune this from open-loop data
#define KFF 200.0f

// Deadband compensation
#define PWM_DEADBAND 15

// Velocity LPF (~5–10 Hz @ 100 Hz loop)
#define ALPHA_LPF 0.15f

class WheelController {
public:
    WheelController(Odometry& odom, MotorDriver& motor);

    void begin();
    void setTargetVelocity(float mps);
    void update(float dt);    // dt in seconds

    void disablePID();        // E-stop safe
    void enablePID();

private:
    Odometry& odom_;
    MotorDriver& motor_;

    float target_vel_;        // m/s
    float vel_raw_;           // m/s
    float vel_filt_;          // m/s

    float error_;
    float prev_error_;
    float integral_;

    float pwm_out_;
    bool enabled_;
};

#endif
