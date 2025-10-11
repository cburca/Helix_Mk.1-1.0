/* ** WHEEL ENCODER HANDLING **
 - Provide wheel velocity and other position-related perameters 
 - Have  signals: ENC_A and ENC_B - 90 degrees out of phase w/1440 pulses per rev
   360 quadrature cycles x 4 edges per cycle, Resolution = 360/1440 = 0.25deg per count
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

#ifndef odometry.h
#define odometry.h

#include <Arduino.h>

class Odometry {
public:
    // Constructor
    Odometry(uint8_t encA, uint8_t encB, float wheelDiameter, int countsPerRev);

    void begin();               // Set up interrupts or pin modes
    void update();              // Called in main loop or timer ISR
    float getVelocity(float dt); // Returns angular velocity (rad/s)
    float getDistance();        // Returns total distance traveled (m)
    void reset();               // Reset tick counter

    // Getters (for ISR access and external use)
    int getEncA() const { return _encA; }
    int getEncB() const { return _encB; }
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
};

// Global ISR handlers (if using attachInterrupt)
void handleLeftEncoderA();
void handleLeftEncoderB();
void handleRightEncoderA();
void handleRightEncoderB();

#endif
