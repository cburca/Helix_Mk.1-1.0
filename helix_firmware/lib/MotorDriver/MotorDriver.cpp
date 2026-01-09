
#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t rpwmPin, uint8_t lpwmPin, uint8_t renPin, uint8_t lenPin, uint8_t isPin)
: _rpwm(rpwmPin), _lpwm(lpwmPin), _ren(renPin), _len(lenPin), _is(isPin), _enabled(false) {}

void MotorDriver::begin(){
    pinMode(_rpwm, OUTPUT);
    pinMode(_lpwm, OUTPUT);
    pinMode(_ren, OUTPUT);
    pinMode(_len, OUTPUT);
    pinMode(_is, INPUT);
}

void MotorDriver::enable(){
    digitalWrite(_ren, HIGH);
    digitalWrite(_len, HIGH);
    _enabled = true;
}

void MotorDriver::disable(){
    digitalWrite(_ren, LOW);
    digitalWrite(_len, LOW);
    analogWrite(_rpwm, 0);
    analogWrite(_lpwm, 0);
    _enabled = false;
}

void MotorDriver::setSpeed(uint16_t speed){
    if (!_enabled)
        return;
    
    speed = constrain(speed, -255, 255); //NEED TO FIND WHAT LIBRARY THIS IS FROM

    if (speed > 0){
        analogWrite(_rpwm, speed);
        analogWrite(_lpwm, 0);
    } else if (speed < 0){
        analogWrite(_rpwm, 0);
        analogWrite(_lpwm, -(speed));
    } else {
        analogWrite(_rpwm, 0);
        analogWrite(_lpwm, 0);
    }
}

int MotorDriver::readCurrent(){
    int adcVal = analogRead(_is);
    return adcVal;
}