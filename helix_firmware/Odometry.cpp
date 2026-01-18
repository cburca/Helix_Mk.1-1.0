#include "core_pins.h"
#include <stdint.h>
#include "Odometry.h"

// --- Constructor ---
Odometry::Odometry(uint8_t encA, uint8_t encB, float wheelDiameter, int countsPerRev, int8_t direction)
    : _encA(encA), _encB(encB), _ticks(0), _lastState(0), _lastUpdateTime(0), 
        _wheelDiameter(wheelDiameter), _countsPerRev(countsPerRev), _prevTicks(0), _dir(direction)  {}

// --- Setup ---
void Odometry::begin() {
    pinMode(_encA, INPUT_PULLUP);
    pinMode(_encB, INPUT_PULLUP);
    _lastState = digitalRead(_encA);
    _lastUpdateTime = millis();
}

// --- Distance Calculation ---
float Odometry::getDistance() {
    float revs = (float)_ticks / _countsPerRev;
    float distance = revs * PI * _wheelDiameter; // circumference * revolutions
    return distance;
}

// --- Velocity Calculation ---
float Odometry::getVelocity(float dt) {
    long currentTicks = _ticks;
    long deltaTicks = currentTicks - _prevTicks;
    _prevTicks = currentTicks;

    float revs = (float)deltaTicks / _countsPerRev;
    float angularVelocity = (revs * 2 * PI) / dt; // rad/s
    _angularVelocity = angularVelocity;
    return angularVelocity;
}

// --- Get Linear Velocity (m/s) ---

/* ONLY WORKS WHEN COMPUTED AFTER getVelocity FUNCTION - NOT TIME DEPENDENT*/

float Odometry::getLinearVelocity() {
    float linearVel = _angularVelocity * (_wheelDiameter / 2); // m/s
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

    long currentTicks = _ticks;
    long deltaTicks = currentTicks - _prevTicks;
    
    _prevTicks = currentTicks;

    // Calculate and store velocity for later retrieval
    _angularVelocity = ((float)deltaTicks / _countsPerRev) * (2 * PI) / dt;
    _linearVelocity = _angularVelocity * (_wheelDiameter / 2.0f);

    _lastUpdateTime = now;
}


// --- Tick Incrementer ---
void Odometry::updateTicks(int8_t delta) {
    _ticks += delta * _dir;
}

/* Encoder ISRs - X4 encoding -> Rising + Falling edges on A & B channels

Result table:
A	B	A << 1	OR B	currentAB
0	0	00  	00	    0b00
0	1	00	    01	    0b01
1	0	10	    10	    0b10
1	1	10	    11	    0b11

*/

void Odometry::leftEncoderISR(){

    static uint8_t lastAB_L = 0b00;

    uint8_t A = digitalRead(LEFT_ENC_A);
    uint8_t B = digitalRead(LEFT_ENC_B);
    uint8_t currentAB = (A << 1) | B;
    int8_t delta = 0;

    if ((lastAB_L == 0b00 && currentAB == 0b01) ||
        (lastAB_L == 0b01 && currentAB == 0b11) ||
        (lastAB_L == 0b11 && currentAB == 0b10) ||
        (lastAB_L == 0b10 && currentAB == 0b00))
        delta = +1;
    else if ((lastAB_L == 0b00 && currentAB == 0b10) ||
             (lastAB_L == 0b10 && currentAB == 0b11) ||
             (lastAB_L == 0b11 && currentAB == 0b01) ||
             (lastAB_L == 0b01 && currentAB == 0b00))
        delta = -1;

    leftOdom.updateTicks(delta);
    lastAB_L = currentAB;
}

void Odometry::rightEncoderISR(){

    static uint8_t lastAB_R = 0;

    uint8_t A = digitalRead(RIGHT_ENC_A);
    uint8_t B = digitalRead(RIGHT_ENC_B);
    uint8_t currentAB = (A << 1) | B;
    int8_t delta = 0;

    if ((lastAB_R == 0b00 && currentAB == 0b01) ||
        (lastAB_R == 0b01 && currentAB == 0b11) ||
        (lastAB_R == 0b11 && currentAB == 0b10) ||
        (lastAB_R == 0b10 && currentAB == 0b00))
        delta = +1;
    else if ((lastAB_R == 0b00 && currentAB == 0b10) ||
             (lastAB_R == 0b10 && currentAB == 0b11) ||
             (lastAB_R == 0b11 && currentAB == 0b01) ||
             (lastAB_R == 0b01 && currentAB == 0b00))
        delta = -1;

    rightOdom.updateTicks(delta);
    lastAB_R = currentAB;
}