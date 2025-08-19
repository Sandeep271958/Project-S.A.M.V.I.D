#include <FastLED.h>

#define NUM_LEDS 16        // Use the maximum number of LEDs needed
#define LED_PIN 6

CRGB leds[NUM_LEDS];

// -------- Mode Definitions --------
#define MODE_STATIC_BLUE     0   // Power ON
#define MODE_RAINBOW_ROTATE  1   // Listening
#define MODE_STATIC_ORANGE   2   // Obstacle
#define MODE_STATIC_RED      3   // Low Battery
#define MODE_BREATH_BLUE     4   // Speaking

int mode = MODE_STATIC_BLUE;   // <-- Change this to select mode

// -------- Variables for animations --------
static int offset = 0;  // For rotating rainbow
bool change = true;     // For breathing effect
int intensity = 55;     // Starting intensity for breathing blue
#define intensity_change 25
#define resting_intensity 55
#define max_intensity 255

void setup() {
  Serial.begin(9600);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 500);
  FastLED.clear();
  FastLED.show();
}

void loop() {
  switch (mode) {
    case MODE_STATIC_BLUE:
      staticBlue();
      break;

    case MODE_RAINBOW_ROTATE:
      rainbowRotate();
      break;

    case MODE_STATIC_ORANGE:
      staticOrange();
      break;

    case MODE_STATIC_RED:
      staticRed();
      break;

    case MODE_BREATH_BLUE:
      breathBlue();
      break;
  }
}

// ---------------- Pattern Functions ----------------

void staticBlue() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 200);
  }
  FastLED.show();
}

void rainbowRotate() {
  int blue_leds = 6;
  int orange_leds = 3;
  int yellow_leds = 3;
  int green_leds = 4;

  for (int i = 0; i < NUM_LEDS; i++) {
    int led_pos = (i + offset) % NUM_LEDS;

    if (i < blue_leds) {
      leds[led_pos] = CRGB(0, 0, 255);
    } else if (i < blue_leds + orange_leds) {
      leds[led_pos] = CRGB(255, 85, 0);
    } else if (i < blue_leds + orange_leds + yellow_leds) {
      leds[led_pos] = CRGB(255, 255, 0);
    } else {
      leds[led_pos] = CRGB(0, 255, 0);
    }
  }
  FastLED.show();
  offset = (offset + 1) % NUM_LEDS;
  delay(100);
}

void staticOrange() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 75, 0);
  }
  FastLED.show();
}

void staticRed() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 0, 0);
  }
  FastLED.show();
}

void breathBlue() {
  if (intensity == resting_intensity) {
    change = true;
  } else if (intensity == max_intensity) {
    change = false;
  }

  if (change) {
    intensity += intensity_change;
  } else {
    intensity -= intensity_change;
  }

  if (intensity < resting_intensity) {
    intensity = resting_intensity;
    change = true;
  } else if (intensity > max_intensity) {
    intensity = max_intensity;
    change = false;
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, intensity);
  }
  FastLED.show();
  delay(30);
}
