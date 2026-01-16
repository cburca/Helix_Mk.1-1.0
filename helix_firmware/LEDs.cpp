#include "LEDs.h"
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
        frontStrip.setPixelColor(i, frontStrip.Color(WHITE));
        backStrip.setPixelColor(i, backStrip.Color(WHITE));
    }
    frontStrip.show();
    backStrip.show();
    delay(1000);

    // Cascade blue inward
    for (int offset = 0; offset < numLeds / 2; offset++) {
        for (int i = 0; i <= offset; i++) {
            frontStrip.setPixelColor(i, frontStrip.Color(BLUE));
            frontStrip.setPixelColor(numLeds - 1 - i, frontStrip.Color(BLUE));
            backStrip.setPixelColor(i, backStrip.Color(BLUE));
            backStrip.setPixelColor(numLeds - 1 - i, backStrip.Color(BLUE));
        }
        frontStrip.show();
        backStrip.show();
        delay(50);
    }
}

void LedController::normalOperation() {
    for (int i = 0; i < numLeds; i++) {
        if (i < 5 || i >= numLeds - 5) {
            frontStrip.setPixelColor(i, frontStrip.Color(WHITE));
            backStrip.setPixelColor(i, backStrip.Color(WHITE));
        } else {
            frontStrip.setPixelColor(i, frontStrip.Color(BLUE));
            backStrip.setPixelColor(i, backStrip.Color(BLUE));
        }
    }
    frontStrip.show();
    backStrip.show();
}

void LedController::handleObstacle() {
    static int brightness = 0;
    static int delta = 5;
    static unsigned long lastUpdate = 0;

    if (millis() - lastUpdate > 30) {
        lastUpdate = millis();
        brightness += delta;
        if (brightness >= 255 || brightness <= 0) delta = -delta;

        for (int i = 0; i < numLeds; i++) {
            // Yellow edges with breathing animation
            if (i < 5 || i >= numLeds - 5) {
                frontStrip.setPixelColor(i, frontStrip.Color(brightness, brightness, 0));
                backStrip.setPixelColor(i, backStrip.Color(brightness, brightness, 0));
            } 
            // Middle section stays solid red
            else {
                frontStrip.setPixelColor(i, frontStrip.Color(255, 0, 0));
                backStrip.setPixelColor(i, backStrip.Color(255, 0, 0));
            }
        }

        frontStrip.show();
        backStrip.show();
    }
}
