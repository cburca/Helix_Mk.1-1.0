#include "PIDController.h"
#include <Arduino.h>

WheelController::WheelController(Odometry& odom, MotorDriver& motor)
    : odom_(odom),
      motor_(motor),
      target_vel_(0.0f),
      vel_raw_(0.0f),
      vel_filt_(0.0f),
      error_(0.0f),
      prev_error_(0.0f),
      integral_(0.0f),
      pwm_out_(0.0f),
      enabled_(false)
{
}

void WheelController::begin() {
    enabled_ = true;
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}

void WheelController::setTargetVelocity(float mps) {
    target_vel_ = mps;
}

void WheelController::update(float dt) {
    if (!enabled_) return;

    // --- Velocity measurement ---
    vel_raw_ = odom_.getStoredLinearVelocity();

    // Low-pass filter
    vel_filt_ = ALPHA_LPF * vel_raw_ + (1.0f - ALPHA_LPF) * vel_filt_;

    // --- Feedforward term ---
    float pwm_ff = KFF * target_vel_;

    // --- PID feedback ---
    error_ = target_vel_ - vel_filt_;
    integral_ += error_ * dt;

    // Anti-windup clamp
    integral_ = constrain(integral_, -3.0f, 3.0f);

    float derivative = (error_ - prev_error_) / dt;
    prev_error_ = error_;

    float pwm_pid =
        KP * error_ +
        KI * integral_ +
        KD * derivative;

    // --- Combine ---
    pwm_out_ = pwm_ff + pwm_pid;

    // Clamp output
    pwm_out_ = constrain(pwm_out_, -255.0f, 255.0f);

    // Deadband compensation
    if (pwm_out_ > 0 && pwm_out_ < PWM_DEADBAND)  pwm_out_ = PWM_DEADBAND;
    if (pwm_out_ < 0 && pwm_out_ > -PWM_DEADBAND) pwm_out_ = -PWM_DEADBAND;

    motor_.setSpeed((int)pwm_out_);
}

void WheelController::disablePID() {
    enabled_ = false;
    motor_.setSpeed(0);

    integral_ = 0.0f;
    prev_error_ = 0.0f;
    pwm_out_ = 0.0f;
}

void WheelController::enablePID() {
    enabled_ = true;
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}