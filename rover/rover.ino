#include <avr/wdt.h>
#include "rover.h"
#include "neopixel_led.h"
#include <Servo.h>

#define MESSAGE_LENGTH 8
#define SERIAL_BAUD 9600

char *buff[MESSAGE_LENGTH];
void (*fcnPtr)() = wait_for_command;
Neopixel_leds n;

Servo left, right;

void setup(){
    Serial.begin(SERIAL_BAUD);
    n.setup_leds();
    wdt_enable(WDTO_1S);

    left.attach(5, 1000, 2000);
    right.attach(6, 1000, 2000);
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
