#include <Adafruit_NeoPixel.h>

// Define pins and number of LEDs
#define FRONT_STRIP_PIN 2
#define BACK_STRIP_PIN 3
#define NUM_LEDS 22  // Adjust this according to the number of LEDs per strip

// Define colors
#define WHITE 255, 255, 255
#define BLUE 0, 0, 255
#define RED 255, 0, 0
#define YELLOW 255, 255, 0
#define OFF 0, 0, 0

Adafruit_NeoPixel frontStrip = Adafruit_NeoPixel(NUM_LEDS, FRONT_STRIP_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel backStrip = Adafruit_NeoPixel(NUM_LEDS, BACK_STRIP_PIN, NEO_GRB + NEO_KHZ800);

bool obstacleDetected = false;

void setup() {
  frontStrip.begin();
  backStrip.begin();
  frontStrip.show(); // Initialize all pixels to 'off'
  backStrip.show(); // Initialize all pixels to 'off'
  
  initialStartup();
}

void loop() {
  if (obstacleDetected) {
    handleObstacle();
  } else {
    normalOperation();
  }
}

void initialStartup() {
  // Turn on both strips bright white
  for (int i = 0; i < NUM_LEDS; i++) {
    frontStrip.setPixelColor(i, frontStrip.Color(WHITE));
    backStrip.setPixelColor(i, backStrip.Color(WHITE));
  }
  frontStrip.show();
  backStrip.show();
  delay(1000); // Delay for 1 second

  // Cascade blue from the edges to the center
  for (int offset = 0; offset < NUM_LEDS / 2; offset++) {
    for (int i = 0; i < offset; i++) {
      frontStrip.setPixelColor(i, frontStrip.Color(BLUE));
      frontStrip.setPixelColor(NUM_LEDS - 1 - i, frontStrip.Color(BLUE));
      backStrip.setPixelColor(i, backStrip.Color(BLUE));
      backStrip.setPixelColor(NUM_LEDS - 1 - i, backStrip.Color(BLUE));
    }
    frontStrip.show();
    backStrip.show();
    delay(50); // Adjust delay for speed of cascading effect
  }
}

void normalOperation() {
  // Set edges to white and middle to blue (front) and red (back)
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < 5 || i >= NUM_LEDS - 5) {  // Edges
      frontStrip.setPixelColor(i, frontStrip.Color(WHITE));
      backStrip.setPixelColor(i, backStrip.Color(WHITE));
    } else {  // Middle
      frontStrip.setPixelColor(i, frontStrip.Color(BLUE));
      backStrip.setPixelColor(i, backStrip.Color(RED));
    }
  }
  frontStrip.show();
  backStrip.show();
}

void handleObstacle() {
  // Change blue sections to red and edges to smooth blinking yellow
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < 5 || i >= NUM_LEDS - 5) {  // Edges
      frontStrip.setPixelColor(i, frontStrip.Color(YELLOW));
      backStrip.setPixelColor(i, backStrip.Color(YELLOW));
    } else {  // Middle (previously blue)
      frontStrip.setPixelColor(i, frontStrip.Color(RED));
      backStrip.setPixelColor(i, backStrip.Color(RED));
    }
  }
  
  frontStrip.show();
  backStrip.show();
  
  // Smooth blinking yellow on edges
  for (int brightness = 0; brightness < 256; brightness += 5) {
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i < 5 || i >= NUM_LEDS - 5) {  // Edges
        frontStrip.setPixelColor(i, frontStrip.Color(brightness, brightness, 0));
        backStrip.setPixelColor(i, backStrip.Color(brightness, brightness, 0));
      }
    }
    frontStrip.show();
    backStrip.show();
    delay(30); // Adjust delay for smoothness of blinking
  }

  for (int brightness = 255; brightness >= 0; brightness -= 5) {
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i < 5 || i >= NUM_LEDS - 5) {  // Edges
        frontStrip.setPixelColor(i, frontStrip.Color(brightness, brightness, 0));
        backStrip.setPixelColor(i, backStrip.Color(brightness, brightness, 0));
      }
    }
    frontStrip.show();
    backStrip.show();
    delay(30); // Adjust delay for smoothness of blinking
  }
}
