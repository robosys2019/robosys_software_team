#ifndef NEOPIXEL_LEDS_H
#define NEOPIXEL_LEDS_H

#include <FastLED.h>

#define LED_DATA 9
#define NUM_LEDS 16
#define BRIGHTNESS 60

class Neopixel_leds{
public:
    void setup_leds();
    void waiting();
    void stuck();
    void deploying();
    void turn_right();
    void turn_left();
    void move_forward();
    void move_reverse();

private:
    CRGB leds[NUM_LEDS];
};

#endif //NEOPIXEL_LEDS_H
