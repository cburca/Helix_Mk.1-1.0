#include <Arduino.h>
#include "Odometry.h"
#include "MotorDriver.h"
#include "LEDs.h"
#include "pins.h"


Odometry leftOdom(LEFT_ENC_A, LEFT_ENC_B, WHEEL_DIAMETER, COUNTS_PER_REV, 1);   // wheel diameter = 160 mm
Odometry rightOdom(RIGHT_ENC_A, RIGHT_ENC_B, WHEEL_DIAMETER, COUNTS_PER_REV, -1);

MotorDriver motorL(BTS7960_1_R_PWM, BTS7960_1_L_PWM, BTS7960_1_R_EN, BTS7960_1_L_EN);
MotorDriver motorR(BTS7960_2_R_PWM, BTS7960_2_L_PWM, BTS7960_2_R_EN, BTS7960_2_L_EN);

LedController ledStrips(NUM_LEDS, LED_STRIP_F, LED_STRIP_R);

// E-Stop Stuff
bool estopActive = false;
int estop = E_STOP;

void estop_press(void){
  estopActive = (digitalRead(estop) == HIGH);
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(100);

    // E-STOP //
    pinMode(estop,INPUT_PULLUP);
    attachInterrupt(estop, estop_press, CHANGE);

    // Encoders //
    // MAYBE DO THIS IN ODOM FILES?
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftOdom.leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), leftOdom.leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightOdom.rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightOdom.rightEncoderISR, CHANGE);

    leftOdom.begin();
    rightOdom.begin();

    // Motors Initialization//
    motorL.begin();
    motorL.enable();
    motorR.begin();
    motorR.enable();

    // LED Strips //
    ledStrips.begin();
    ledStrips.initialStartup();
    ledStrips.normalOperation();

    Serial.println("==== ODOMETRY TEST START ====");
    Serial.println("Encoders ready. Rotate wheels or run motors.");
    Serial.println();

    motorL.setSpeed(80);
    motorR.setSpeed(-80);
}

// --- Loop ---
void loop() {

//////// Single-Wheel Rotation Test ///////////
    // if(leftOdom.getTicks() > 1440){
    //     motorL.disable();
    // }
    // else if(rightOdom.getTicks() > 1440){
    //     motorR.disable();
    // }
///////////////////////////////////////////////

    static unsigned long lastTime = millis();
    unsigned long now = millis();

    if (now - lastTime >= 500) { // every 0.5 seconds
        float dt = (now - lastTime) / 1000.0f; // convert to seconds
        lastTime = now;

        // 1. Distance test
        float leftDist = leftOdom.getDistance();
        float rightDist = rightOdom.getDistance();

        // 2. Velocity tests
        float leftAngVel = leftOdom.getVelocity(dt);
        float rightAngVel = rightOdom.getVelocity(dt);

        float leftLinVel = leftOdom.getLinearVelocity(dt);
        float rightLinVel = rightOdom.getLinearVelocity(dt);

        // 3. Print results
        Serial.println("------ Encoder Report ------");
        Serial.print("Ticks [L,R]: ");
        Serial.print(leftOdom.getTicks()); Serial.print(", ");
        Serial.println(rightOdom.getTicks());

        Serial.print("Distance [m] [L,R]: ");
        Serial.print(leftDist, 5); Serial.print(", ");
        Serial.println(rightDist, 5);

        Serial.print("Angular Vel [rad/s] [L,R]: ");
        Serial.print(leftAngVel, 3); Serial.print(", ");
        Serial.println(rightAngVel, 3);

        Serial.print("Linear Vel [m/s] [L,R]: ");
        Serial.print(leftLinVel, 3); Serial.print(", ");
        Serial.println(rightLinVel, 3);

        Serial.println("-----------------------------\n");
    }

    if(estopActive == true){
        motorL.disable();
        motorR.disable();

        while(estopActive){
            ledStrips.handleObstacle();
        }

        motorL.enable();
        motorR.enable();
        motorL.setSpeed(50);
        motorR.setSpeed(-50);

        ledStrips.normalOperation();
    }
}