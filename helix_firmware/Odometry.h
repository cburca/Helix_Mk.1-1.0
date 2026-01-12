/* ** WHEEL ENCODER HANDLING **
 - Provide wheel velocity and other position-related perameters 
 - Have  signals: ENC_A and ENC_B - 90 degrees out of phase w/1440 pulses per rev
   60:1 gearbox, so counts per wheel rev = 1440 x 60 = 86,400 ticks
 - Read encoder pulses from both motors
    - Track: 
        - ∆ticks over time 
        - Wheel velocities (rad/s, m/s)
        - Pose (x, y, theta)
    - Provide higher-level controller:
        - Distance per wheel
        - Total Displacement
        - Orientation
*/
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "pins.h"
#include <Arduino.h>


const int MOTOR_CPR = 1440;
const int GEAR_RATIO = 60;

class Odometry {
public:
    // Constructor
    Odometry(uint8_t encA, uint8_t encB, float wheelDiameter, int countsPerRev);

    void begin();               // Set up interrupts or pin modes
    float getVelocity(float dt); // Returns angular velocity (rad/s)
    float getLinearVelocity(float dt); // Returns linear velocity (m/s)
    float getDistance();        // Returns total distance traveled (m)
    void update();          // Optional for rn, use later for pre-calcualting & storing pose estimates & velocities
    void reset();               // Reset tick counter

    // Methods to grab stored pose/vel data from update function once used
    float getStoredAngularVelocity() const { return _angularVelocity; }
    float getStoredLinearVelocity() const { return _linearVelocity; }

    // Getters (for ISR access and external use)
    uint8_t getEncA() const { return _encA; }
    uint8_t getEncB() const { return _encB; }
    long getTicks() const { return _ticks; }

    // ISR helper — updates ticks safely
    void updateTicks(bool direction);

private:
    uint8_t _encA, _encB;
    volatile long _ticks;       // Encoder tick count
    int _lastState;
    unsigned long _lastUpdateTime;
    float _wheelDiameter;
    int _countsPerRev;
    float _angularVelocity = 0.0f;
    float _linearVelocity = 0.0f;
    long _prevTicks;
};

// Global ISR handlers (if using attachInterrupt)
void leftEncoderISR();
void rightEncoderISR();

#endif
