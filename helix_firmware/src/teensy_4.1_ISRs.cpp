#include "Arduino.h"
#include "odometry.h"


///////////////////////////
// ** My Encoder ISRs ** //
///////////////////////////

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