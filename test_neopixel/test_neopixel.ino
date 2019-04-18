#include <Adafruit_NeoPixel.h>

#define DATA 9
#define PIXELS 16

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELS, DATA, NEO_KHZ800);

void setup(){
	strip.begin();
	strip.show();
}

void loop(){
	for(int i=0; i<PIXELS; i++){
		strip.setPixelColor(i, strip.Color(127, 127, 127));
	}
	strip.show();
	delay(100);
}
