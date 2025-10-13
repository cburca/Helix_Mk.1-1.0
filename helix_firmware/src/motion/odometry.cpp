#include "odometry.h"
#include "pins.h"

// Constructor
Odometry::Odometry(uint8_t encA, uint8_t encB, float wheelDiameter, int countsPerRev)
    : _encA(encA), _encB(encB), _wheelDiameter(wheelDiameter), _countsPerRev(countsPerRev),
        _ticks(0), _lastState(0), _lastUpdateTime(0) {}


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

