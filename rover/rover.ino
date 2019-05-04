#include <avr/wdt.h>
#include "rover.h"
#include "neopixel_led.h"
#include <Servo.h>

#define MESSAGE_LENGTH 3
#define SERIAL_BAUD 9600

char *buff[MESSAGE_LENGTH];
void (*fcnPtr)() = wait_for_command;
Neopixel_leds n;

Servo left, right;

short angle;
byte distance;

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

    // 2 bytes for 0-360 degrees of rotation
    // 1 byte for 0-255 inches (up to ~21 feet, more than enough)
    angle = buff;
    n.turn_left();

    //

    Serial.print(255); // command for done
}

void check_ir(){

}
