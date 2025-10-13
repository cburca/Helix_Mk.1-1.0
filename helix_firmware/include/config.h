//PINMODES, BAUDRATES, ETC
#include "Arduino.h"
#include "pins.h"
#include "odometry.h"


void enc_begin(){
    // Left Enc
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), handleLeftEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), handleLeftEncoderB, CHANGE);
    // Right Enc
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), handleRightEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), handleRightEncoderB, CHANGE);
}