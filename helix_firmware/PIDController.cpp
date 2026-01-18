#include "PIDController.h"

WheelController::WheelController(Odometry& odom, MotorDriver& motor)
    : odom_(odom),
      motor_(motor),
      target_vel_(0),
      measured_vel_(0),
      pwm_out_(0),
      pid_(&measured_vel_filtered, &pwm_out_, &target_vel_,
           Kp, Ki, Kd, DIRECT) // example gains
{
}

void WheelController::begin() {
    pid_.SetOutputLimits(-255, 255);
    pid_.SetSampleTime(10); // To match sampling time in void loop (100Hz)
    pid_.SetMode(AUTOMATIC);

}

void WheelController::setTargetVelocity(float met_per_sec) {
    target_vel_ = met_per_sec;
}

void WheelController::update() {
    //measured_vel_ = odom_.getStoredLinearVelocity();   // m/s
    measured_vel_ = 0.8f * measured_vel_ + 0.2f * odom_.getStoredLinearVelocity();    // Velocity Smoothing 
    measured_vel_filtered = ALPHA_LPF * measured_vel_filtered + (1 - ALPHA_LPF) * measured_vel_;    // 10 Hz cutoff LPF
    pid_.Compute();
    /* Deadband Compensation */
    if (pwm_out_ > 0 && pwm_out_ < 40) pwm_out_ = 40;
    if (pwm_out_ < 0 && pwm_out_ > -40) pwm_out_ = -40;
    motor_.setSpeed((int)pwm_out_);
}

void WheelController::disablePID() {    // For when motors have emergency stop (Ie. V = 0) & risk of integral windup: target stays nonzero, measured is stuck at zero, controller keeps accumulating error
  pid_.SetMode(MANUAL);
  pwm_out_ = 0;
  motor_.setSpeed(0);
}

void WheelController::enablePID() {
  pid_.SetMode(MANUAL);   // Force reset of PID parameters (Ie. Integral Windup)
  pid_.SetMode(AUTOMATIC);
}