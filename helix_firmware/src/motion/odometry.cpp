#include "odometry.h"
#include "pins.h"

// --- Constructor ---
Odometry::Odometry(uint8_t encA, uint8_t encB, float wheelDiameter, int countsPerRev)
    : _encA(encA), _encB(encB), _wheelDiameter(wheelDiameter), _countsPerRev(countsPerRev),
      _ticks(0), _lastState(0), _lastUpdateTime(0) {}

// --- Setup ---
void Odometry::begin() {
    pinMode(_encA, INPUT_PULLUP);
    pinMode(_encB, INPUT_PULLUP);
    _lastState = digitalRead(_encA);
}

// --- Tick Incrementer ---
void Odometry::updateTicks(bool direction) {
    if (direction) _ticks++;
    else _ticks--;
}

// --- Distance Calculation ---
float Odometry::getDistance() {
    float revs = (float)_ticks / _countsPerRev;
    float distance = revs * PI * _wheelDiameter; // circumference * revolutions
    return distance;
}

// --- Velocity Calculation ---
float Odometry::getVelocity(float dt) {
    static long prevTicks = 0;
    long currentTicks = _ticks;
    long deltaTicks = currentTicks - prevTicks;
    prevTicks = currentTicks;

    float revs = (float)deltaTicks / _countsPerRev;
    float angularVelocity = (revs * 2 * PI) / dt; // rad/s
    return angularVelocity;
}

// --- Get Linear Velocity (m/s) ---
float Odometry::getLinearVelocity(float dt) {
    float angularVel = getVelocity(dt);
    float linearVel = angularVel * (_wheelDiameter / 2.0f);
    return linearVel;
}

// --- Reset ---
void Odometry::reset() {
    _ticks = 0;
}

// --- Upodate --- //
// For later also include Pose Estimates (x, y, theta)
void Odometry::update() {
    unsigned long now = micros();
    float dt = (now - _lastUpdateTime) / 1e6; // seconds
    if (dt <= 0) return;

    static long prevTicks = 0;
    long currentTicks = _ticks;
    long deltaTicks = currentTicks - prevTicks;
    prevTicks = currentTicks;

    // Calculate and store velocity for later retrieval
    _angularVelocity = ((float)deltaTicks / _countsPerRev) * (2 * PI) / dt;
    _linearVelocity = _angularVelocity * (_wheelDiameter / 2.0f);

    _lastUpdateTime = now;
}