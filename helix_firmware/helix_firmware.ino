#include <Arduino.h>
#include "pins.h"
#include "odometry.h"

// TEST MESSAGE //

// Create odometry objects
Odometry leftOdom(LEFT_ENC_A, LEFT_ENC_B, 0.096, 1440);   // wheel diameter = 96 mm
//Odometry rightOdom(RIGHT_ENC_A, RIGHT_ENC_B, 0.096, 1440);

// --- ISR handlers ---
void leftEncoderISR() {
    bool A = digitalRead(LEFT_ENC_A);
    bool B = digitalRead(LEFT_ENC_B);
    bool direction = (A == B);  // Basic quadrature direction rule
    leftOdom.updateTicks(direction);
}

// void rightEncoderISR() {
//     bool A = digitalRead(RIGHT_ENC_A);
//     bool B = digitalRead(RIGHT_ENC_B);
//     bool direction = (A == B);
//     rightOdom.updateTicks(direction);
// }

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("==== ODOMETRY TEST START ====");

    leftOdom.begin();
    //rightOdom.begin();

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

    Serial.println("Encoders ready. Rotate wheels or run motors.");
    Serial.println();
}

// --- Loop ---
void loop() {
    static unsigned long lastTime = millis();
    unsigned long now = millis();

    if (now - lastTime >= 500) { // every 0.5 seconds
        float dt = (now - lastTime) / 1000.0f; // convert to seconds
        lastTime = now;

        // 1. Distance test
        float leftDist = leftOdom.getDistance();
        //float rightDist = rightOdom.getDistance();

        // 2. Velocity tests
        float leftAngVel = leftOdom.getVelocity(dt);
        //float rightAngVel = rightOdom.getVelocity(dt);
        float leftLinVel = leftOdom.getLinearVelocity(dt);
        //float rightLinVel = rightOdom.getLinearVelocity(dt);

        // 3. Print results
        Serial.println("------ Encoder Report ------");
        Serial.print("Ticks [L,R]: ");
        Serial.print(leftOdom.getTicks()); Serial.print(", ");
        //Serial.println(rightOdom.getTicks());
        delay(100);
        Serial.print("Distance [m] [L,R]: ");
        Serial.print(leftDist, 5); Serial.print(", ");
        //Serial.println(rightDist, 5);
        delay(100);
        Serial.print("Angular Vel [rad/s] [L,R]: ");
        Serial.print(leftAngVel, 3); Serial.print(", ");
        //Serial.println(rightAngVel, 3);
        delay(100);
        Serial.print("Linear Vel [m/s] [L,R]: ");
        Serial.print(leftLinVel, 3); Serial.print(", ");
        //Serial.println(rightLinVel, 3);
        delay(100);
        Serial.println("-----------------------------\n");
    }
}