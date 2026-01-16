#ifndef LEDS_H
#define LEDS_H

#include <Adafruit_NeoPixel.h>

#define NUM_LEDS 22  // Adjust this according to the number of LEDs per strip

#define WHITE 255, 255, 255
#define BLUE 0, 0, 255
#define RED 255, 0, 0
#define YELLOW 255, 255, 0
#define OFF 0, 0, 0

// A class to encapsulate LED behavior
class LedController {
public:
    LedController(int numLeds, int frontPin, int backPin);

    void begin();              // Initialize strips
    void initialStartup();     // Startup animation
    void normalOperation();    // Normal running pattern
    void handleObstacle();     // Obstacle response

    void setObstacleDetected(bool state);   // Setter
    bool getObstacleDetected(void) const {return obstacleDetected;};   // Getter

private:
    Adafruit_NeoPixel frontStrip;
    Adafruit_NeoPixel backStrip;
    int numLeds;
    bool obstacleDetected;
};

#endif
