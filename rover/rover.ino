#include <avr/wdt.h>

#define MESSAGE_LENGTH 8
#define SERIAL_BAUD 9600

char *buff[MESSAGE_LENGTH];

void setup(){
	Serial.begin(SERIAL_BAUD);
	wdt_enable(WDTO_1S);
}

void loop(){
	if(serial.available()){
		serial.readBytes(*buff, MESSAGE_LENGTH);
	}
	wdt_reset();
	delay(10);
}
