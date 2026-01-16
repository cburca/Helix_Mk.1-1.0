/* ** WHEEL ENCODER HANDLING **
 - Provide wheel velocity and other position-related perameters 
 - Have  signals: ENC_A and ENC_B - 90 degrees out of phase w/1440 pulses per rev AT THE WHEEL
 - Read encoder pulses from both motors
    - Track: 
        - ∆ticks over time 
        - Wheel velocities (rad/s, m/s)
        - Pose (x, y, theta)
    - Provide higher-level controller:
        - Distance per wheel
        - Total Displacement
        - Orientation
 - Max Expected linear velocity:
    100rpm / 60 = 1.667 [rev/s]
    Circumfrence = Pi * (160mm / 2)
    v = 1.667 [rev/s] x 0.5027 [m/rev] = 0.84 m/s
    So rough expected lin. vel. = 0.8-0.9 m/s
*/
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "pins.h"
#include <Arduino.h>


const int COUNTS_PER_REV = 1440;
const float WHEEL_DIAMETER = 0.160;

class Odometry {
public:
    // Constructor
    Odometry(uint8_t encA, uint8_t encB, float wheelDiameter, int countsPerRev, int8_t direction);

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
    void updateTicks(int8_t delta);

    // Global ISR handlers (if using attachInterrupt)
    static void leftEncoderISR(void);
    static void rightEncoderISR(void);

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
    int8_t _dir;
};

extern Odometry leftOdom;
extern Odometry rightOdom;


#endif
