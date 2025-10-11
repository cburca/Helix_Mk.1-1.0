#include "leds.h"
#include "pins.h"  // for NUM_LEDS, LED_STRIP_F, LED_STRIP_R
#include <Arduino.h>  // for delay()

// Constructor: initializes member variables
LedController::LedController(int numLeds, int frontPin, int backPin):
      frontStrip(numLeds, frontPin, NEO_GRB + NEO_KHZ800),
      backStrip(numLeds, backPin, NEO_GRB + NEO_KHZ800),
      numLeds(numLeds),
      obstacleDetected(false) {}

void LedController::begin() {
    frontStrip.begin();
    backStrip.begin();
    frontStrip.show();
    backStrip.show();
}

void LedController::setObstacleDetected(bool state) {
    obstacleDetected = state;
}

void LedController::initialStartup() {
    // Turn both strips bright white
    for (int i = 0; i < numLeds; i++) {
        frontStrip.setPixelColor(i, frontStrip.Color(255, 255, 255));
        backStrip.setPixelColor(i, backStrip.Color(255, 255, 255));
    }
    frontStrip.show();
    backStrip.show();
    delay(1000);

    // Cascade blue inward
    for (int offset = 0; offset < numLeds / 2; offset++) {
        for (int i = 0; i <= offset; i++) {
            frontStrip.setPixelColor(i, frontStrip.Color(0, 0, 255));
            frontStrip.setPixelColor(numLeds - 1 - i, frontStrip.Color(0, 0, 255));
            backStrip.setPixelColor(i, backStrip.Color(0, 0, 255));
            backStrip.setPixelColor(numLeds - 1 - i, backStrip.Color(0, 0, 255));
        }
        frontStrip.show();
        backStrip.show();
        delay(50);
    }
}

void LedController::normalOperation() {
    for (int i = 0; i < numLeds; i++) {
        if (i < 5 || i >= numLeds - 5) {
            frontStrip.setPixelColor(i, frontStrip.Color(255, 255, 255));
            backStrip.setPixelColor(i, backStrip.Color(255, 255, 255));
        } else {
            frontStrip.setPixelColor(i, frontStrip.Color(0, 0, 255));
            backStrip.setPixelColor(i, backStrip.Color(255, 0, 0));
        }
    }
    frontStrip.show();
    backStrip.show();
}

void LedController::handleObstacle() {
    for (int i = 0; i < numLeds; i++) {
        if (i < 5 || i >= numLeds - 5) {
            frontStrip.setPixelColor(i, frontStrip.Color(255, 255, 0));
            backStrip.setPixelColor(i, backStrip.Color(255, 255, 0));
        } else {
            frontStrip.setPixelColor(i, frontStrip.Color(255, 0, 0));
            backStrip.setPixelColor(i, backStrip.Color(255, 0, 0));
        }
    }
    frontStrip.show();
    backStrip.show();

    // Smooth yellow breathing
    for (int b = 0; b <= 255; b += 5) {
        for (int i = 0; i < numLeds; i++) {
            if (i < 5 || i >= numLeds - 5) {
                frontStrip.setPixelColor(i, frontStrip.Color(b, b, 0));
                backStrip.setPixelColor(i, backStrip.Color(b, b, 0));
            }
        }
        frontStrip.show();
        backStrip.show();
        delay(30);
    }

    for (int b = 255; b >= 0; b -= 5) {
        for (int i = 0; i < numLeds; i++) {
            if (i < 5 || i >= numLeds - 5) {
                frontStrip.setPixelColor(i, frontStrip.Color(b, b, 0));
                backStrip.setPixelColor(i, backStrip.Color(b, b, 0));
            }
        }
        frontStrip.show();
        backStrip.show();
        delay(30);
    }
}
