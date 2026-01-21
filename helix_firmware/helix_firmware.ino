#include <Arduino.h>
#include "Odometry.h"
#include "MotorDriver.h"
#include "LEDs.h"
#include "pins.h"
#include "PIDController.h"


Odometry leftOdom(LEFT_ENC_A, LEFT_ENC_B, WHEEL_DIAMETER, COUNTS_PER_REV, 1);   // wheel diameter = 160 mm
Odometry rightOdom(RIGHT_ENC_A, RIGHT_ENC_B, WHEEL_DIAMETER, COUNTS_PER_REV, -1);

MotorDriver motorL(BTS7960_1_R_PWM, BTS7960_1_L_PWM, BTS7960_1_R_EN, BTS7960_1_L_EN);
MotorDriver motorR(BTS7960_2_R_PWM, BTS7960_2_L_PWM, BTS7960_2_R_EN, BTS7960_2_L_EN);

// Instantiate PID control objects (left/right wheels) after setting up odom & motors
WheelController leftPID(leftOdom, motorL);
WheelController rightPID(rightOdom, motorR);


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
    delay(10);

    // Motors Init. //
    motorL.begin();
    motorL.enable();
    motorR.begin();
    motorR.enable();

    // PID Control Init. //
    leftPID.begin();
    rightPID.begin();
    delay(10);

    // LED Strips //
    ledStrips.begin();
    ledStrips.initialStartup();
    ledStrips.normalOperation();

    Serial.println("==== ODOMETRY TEST START ====");
    Serial.println("Encoders ready. Rotate wheels or run motors.");
    Serial.println();

    // OPEN-LOOP MOTOR CONTROL TESTING //
    // motorL.setSpeed(80);
    // motorR.setSpeed(80);

    // // // CLOSED-LOOP MOTOR CONTROL TESTING //
    leftPID.setTargetVelocity(0.3);
    rightPID.setTargetVelocity(0.3);
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
    unsigned long now = millis();
    static unsigned long lastDiag = millis();
    static unsigned long lastPID = millis();

    if (now - lastPID >= 100) {  // 100 Hz
        float dt = (now - lastPID) / 1000.0f;
        lastPID = now;

        leftOdom.update();
        rightOdom.update();

        leftPID.update(dt);
        rightPID.update(dt);

        // float leftLinVel = leftOdom.getStoredLinearVelocity();
        // float rightLinVel = rightOdom.getStoredLinearVelocity();

        // Serial.println("------ Encoder Report ------");
        // Serial.print("Linear Vel [m/s] [L,R]: ");
        // Serial.print(leftLinVel, 3); Serial.print(", ");
        // Serial.println(rightLinVel, 3);
        // Serial.println("-----------------------------\n");
    }

    if (now - lastDiag >= 300) { // every 0.3 seconds
        lastDiag = now;

    //     // // 1. Distance test
        float leftDist = leftOdom.getDistance();
        float rightDist = rightOdom.getDistance();

    //     // // 2. Velocity tests
        float leftAngVel = leftOdom.getStoredAngularVelocity();
        float rightAngVel = rightOdom.getStoredAngularVelocity();

        float leftLinVel = leftOdom.getStoredLinearVelocity();
        float rightLinVel = rightOdom.getStoredLinearVelocity();

    //     // 3. Print results
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

        leftPID.disablePID();
        rightPID.disablePID();

        while(estopActive){
            ledStrips.handleObstacle();
        }

        motorL.enable();
        motorR.enable();
        // motorL.setSpeed(50);     // <--- Open Loop
        // motorR.setSpeed(-50);
        leftPID.enablePID();        // <--- Closed Loop - need to make sure no integral windup
        rightPID.enablePID();

        ledStrips.normalOperation();
    }
}