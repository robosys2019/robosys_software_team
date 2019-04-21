#include <avr/wdt.h>
#include "rover.h"
#include "neopixel_led.h"

#define MESSAGE_LENGTH 8
#define SERIAL_BAUD 9600

char *buff[MESSAGE_LENGTH];
void (*fcnPtr)() = wait_for_command;
Neopixel_leds n;

void setup(){
	Serial.begin(SERIAL_BAUD);
	n.setup_leds();
	wdt_enable(WDTO_1S);
}

void loop(){
	fcnPtr();
}

void wait_for_command(){
	n.waiting();
	if(Serial.available()){
		Serial.readBytes(*buff, MESSAGE_LENGTH);
		fcnPtr = command;
	}
	wdt_reset();
	delay(10);
}

void command(){
	// Parse the movement
	n.turn_left();

}

void check_ir(){

}
