#include "neopixel_led.h"

void Neopixel_leds::setup_leds(){
    FastLED.addLeds<NEOPIXEL, LED_DATA>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);

    for(int i=0; i<NUM_LEDS; i++){
        leds[i] = CRGB::Blue;
    }
    FastLED.show();

    delay(100);
    
    for(int i=0; i<NUM_LEDS; i++){
        leds[i] = CRGB::Black;
    }
    FastLED.show();
}

void Neopixel_leds::waiting(){
    for(int i=0; i<NUM_LEDS; i++){
        leds[i] = CRGB::Green;
    }
    FastLED.show();
}

void Neopixel_leds::stuck(){
    for(int i=0; i<NUM_LEDS; i++){
        leds[i] = CRGB::Red;
    }
    FastLED.show();
}

void Neopixel_leds::deploying(){
    for(int i=0; i<NUM_LEDS; i++){
        leds[i] = CRGB::Blue;
    }
    FastLED.show();
}

void Neopixel_leds::turn_right(){
    for(int i=NUM_LEDS/2; i<NUM_LEDS; i++){
        leds[i] = CRGB::Yellow;
    }
    FastLED.show();
}

void Neopixel_leds::turn_left(){
    for(int i=0; i<NUM_LEDS/2; i++){

        leds[i] = CRGB::Yellow;
    }
    FastLED.show();
}


