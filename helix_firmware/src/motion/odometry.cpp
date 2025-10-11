#include "odometry.h"
#include "pins.h"

// Constructor
Odometry::Odometry(uint8_t encA, uint8_t encB, float wheelDiameter, int countsPerRev)
: _encA(encA), _encB(encB), _wheelDiameter(wheelDiameter), _countsPerRev(countsPerRev),
  _ticks(0), _lastState(0), _lastUpdateTime(0) {}


// Just as placeholder so interrupt handlers in odom won't error out
Odometry leftOdom(LEFT_ENC_A, LEFT_ENC_B, 0.096, 1440);
Odometry rightOdom(RIGHT_ENC_A, RIGHT_ENC_B, 0.096, 1440);

// ** My Encoder ISRs ** //

void handleLeftEncoderA() {
    int a = digitalRead(leftOdom.getEncA());
    int b = digitalRead(leftOdom.getEncB());
    leftOdom.updateTicks(a == b);  // If in phase, forward; else reverse
}

void handleLeftEncoderB() {
    int a = digitalRead(leftOdom.getEncA());
    int b = digitalRead(leftOdom.getEncB());
    leftOdom.updateTicks(a != b);  // Reverse logic for channel B
}

void handleRightEncoderA() {
    int a = digitalRead(rightOdom.getEncA());
    int b = digitalRead(rightOdom.getEncB());
    rightOdom.updateTicks(a == b);
}

void handleRightEncoderB() {
    int a = digitalRead(rightOdom.getEncA());
    int b = digitalRead(rightOdom.getEncB());
    rightOdom.updateTicks(a != b);
}

void Odometry::begin() {
    pinMode(_encA, INPUT_PULLUP);
    pinMode(_encB, INPUT_PULLUP);

    // Read initial state
    _lastState = digitalRead(_encA);
}

void Odometry::update() {
    // This function can handle filtering, velocity calc, etc.



}

float Odometry::getVelocity(float dt) {
    static long prevTicks = 0;
    long currentTicks = _ticks;
    long deltaTicks = currentTicks - prevTicks;
    prevTicks = currentTicks;

    float revs = (float)deltaTicks / _countsPerRev;
    float angularVelocity = (revs * 2 * PI) / dt; // rad/s
    return angularVelocity;
}

float Odometry::getDistance() {
    float revs = (float)_ticks / _countsPerRev;
    float distance = revs * PI * _wheelDiameter;
    return distance;
}

void Odometry::reset() {
    _ticks = 0;
}

void Odometry::updateTicks(bool direction) {
    if (direction) _ticks++;
    else _ticks--;
}

