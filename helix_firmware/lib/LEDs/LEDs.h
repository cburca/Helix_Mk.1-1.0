#ifndef LEDS_H
#define LEDS_H

#include <Adafruit_NeoPixel.h>

// A class to encapsulate LED behavior
class LedController {
public:
    LedController(int numLeds, int frontPin, int backPin);

    void begin();              // Initialize strips
    void initialStartup();     // Startup animation
    void normalOperation();    // Normal running pattern
    void handleObstacle();     // Obstacle response

    void setObstacleDetected(bool state);

private:
    Adafruit_NeoPixel frontStrip;
    Adafruit_NeoPixel backStrip;
    int numLeds;
    bool obstacleDetected;
};

#endif
