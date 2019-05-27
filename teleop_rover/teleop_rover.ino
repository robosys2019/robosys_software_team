#include <Servo.h>

#define SERIAL_BAUD 9600

#define FORWARD 120
#define REVERSE 60

Servo left, right, r1, r2;

byte left_speed=90, right_speed=90;

char cmd;

void setup(){
    Serial.begin(SERIAL_BAUD);
    
    r1.attach(3);
    r2.attach(4);

    left.attach(5, 1000, 2000);
    right.attach(6, 1000, 2000);
    left.write(90);
    right.write(90);

    r1.write(180);
    r2.write(180);
}

void loop(){
        if(Serial.available()){
        cmd = Serial.read();
        Serial.println(cmd);

        switch(cmd){
            case 'w':
            left_speed = FORWARD;
            right_speed = FORWARD;
            break;
            case 'a':
            left_speed = REVERSE;
            right_speed = FORWARD;
            break;
            case 's':
            left_speed = FORWARD;
            right_speed = REVERSE;
            break;
            case 'd':
            left_speed = REVERSE;
            right_speed = REVERSE;
            break;
            case ' ':
            left_speed = 90;
            right_speed = 90;
            break;
            case '1':
            r1.write(30);
            delay(1000);
            r1.write(180);
            break;
            case '2':
            r2.write(30);
            delay(1000);
            r2.write(180);
            break;
        }
        
    }
    delay(10);

    left.write(left_speed);
    right.write(right_speed);
}
