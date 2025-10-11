// Disable motor enable pins when e-stop interrupts - provides motor braking

#ifndef motor_driver.h
#define motor_driver.h

#include <Arduino.h>    

class MotorDriver {
public:
    // constructor
    MotorDriver(uint8_t R_pwmPin, uint8_t L_pwmPin, uint8_t R_enPin, uint8_t L_enPin, uint8_t isPin);

    void begin();   // To initialize pins
    void setSpeed(uint16_t speed); // -255 to +255 where negative is reverse direction
    void enable();
    void disable();
    int readCurrent();  // Returns ADC reading or mA

private:
    uint8_t _rpwm, _lpwm, _ren, _len, _is;
    bool _enabled;

};

#endif
