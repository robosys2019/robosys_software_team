#include <avr/wdt.h>
#include "rover.h"
#include "neopixel_led.h"
#include <Servo.h>

#define MESSAGE_LENGTH 3
#define SERIAL_BAUD 9600

#define FORWARD 120
#define REVERSE 60

char *buff[MESSAGE_LENGTH];
void (*fcnPtr)() = wait_for_command;
Neopixel_leds n;

Servo left, right;

unsigned long time;
unsigned long movement_time;
unsigned long time_moved;
short angle;
byte distance;

byte left_speed, right_speed;

void setup(){
    Serial.begin(SERIAL_BAUD);
    n.setup_leds();

    left.attach(5, 1000, 2000);
    right.attach(6, 1000, 2000);
    n.waiting();
}

void loop(){
    fcnPtr();
}

void wait_for_command(){
    if(Serial.available()){
        Serial.readBytes(*buff, MESSAGE_LENGTH);
        Serial.print(127); // command for received
        fcnPtr = parse_command;
    }
    delay(10);
}

void parse_command(){
    // Parse the movement

    // 2 bytes for 0-360 degrees of rotation
    // 1 byte for 0-255 inches (up to ~21 feet, more than enough)
    angle = short(buff[1])+short(buff[0])-180;
    distance = byte(buff[2]);

    if(angle>0){
        n.turn_left();
        left_speed = REVERSE;
        right_speed = FORWARD;
        fcnPtr = turn;
    }
    else{
        n.turn_right();
        left_speed = FORWARD;
        right_speed = REVERSE;
        fcnPtr = turn;
    }
}

void turn(){
    movement_time = abs(angle);

    //Start Turning
    left.write(left_speed);
    right.write(right_speed);
    time = millis();

    while((millis()-time)<movement_time){} // wait until it has turned enough

    //Stop Turning
    left.write(90);
    right.write(90);

    n.move_forward();
    fcnPtr = move_forward;
}

void move_forward(){
    left_speed = FORWARD;
    right_speed = FORWARD;

    left.write(left_speed);
    right.write(right_speed);
    time = millis();

    while((millis()-time)<movement_time){
        if(check_ir){
            left.write(90);
            right.write(90);
            time_moved = millis()-time;

            n.stuck();

            n.move_reverse();
            fcnPtr = move_reverse;
        }
    }

    //Stop Moving
    left.write(90);
    right.write(90);

    Serial.print(255);

    n.waiting();
    fcnPtr = wait_for_command;
}

void move_reverse(){
    left_speed = REVERSE;
    right_speed = REVERSE;

    left.write(left_speed);
    right.write(right_speed);
    time = millis();

    while((millis()-time)<time_moved){}

    //Stop Moving
    left.write(90);
    right.write(90);

    Serial.print(90);

    n.waiting();
    fcnPtr = wait_for_command;
}

bool check_ir(){

    // Code for checking the IR sensors
    return false;
}
