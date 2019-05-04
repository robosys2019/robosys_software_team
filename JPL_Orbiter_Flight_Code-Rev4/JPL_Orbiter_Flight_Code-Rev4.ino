/***********************************************************************************************
 *  Title: 2019 JPL Orbiter Flight Code-rev4
 *  Description: This  structure template contains a SENSE-THINK-ACT data flow to 
 *  allow a  Robot to perform a sequence of meta-behaviors in soft-real-time based on direct
 *  text commands from a human operator.
 *  Robot Name?: JPL Orbiter
 *  What does code do?: Wait 5 seconds,close drop hook,move downrange till see target,drop lander, 
 *  wait 5 sec, return home, wait 5 seconds, close drop hook, repeat sequence
 *  Hardware warnings:  Turn  motor power on AFTER starting this code
 *  Created by D.Barrett March 2019
 *  ******************************************************************************************** */

// TODO section: Add all future work to do here 
//  add sensor averaging to sense section, add motor speed to act section?
//  strip out printlines? 

//==============================================================================================
// Load supporting Arduino Libraries
//=============================================================================================
#include <Servo.h>               //example of loading ServoMotors library

//==============================================================================================
// Create and initialize global variables, objects and constants (containers for all data)
//===============================================================================================
Servo dropServo;                 // create servo object to control a lander release servomotor
Servo tractionServo;             // create servo object to control a main traction drive motor

#define aliveLED  13             //create a name for "robot alive" blinky light pin
#define soundPin  8              //create a name for the pin connected to state speaker
#define IRdownRange  A4          //create a name for pin connected to down range IR Sensor
#define IRhomeRange  A5          //create a name for pin connected to home range IR Sensor
#define hookOpen  95             // exp measured hook servo positions
#define hookClosed 45
#define moveDown 30               // exp measured motor servo direction
#define stopOrbiter 90
#define moveHome 130
#define pauseSound 1000          // add sound definitions
#define runSound 70
#define returnSound 3000

int beepSound = 100;                      // add a sound vaiable
int homeRange = 0;                       // TODO  temp coding range variable
int downRange = 0;
int dropServoAngle = hookOpen;            //create global dropServoAngle
int dropServoAngleOld = hookOpen;   
int tractionMotorSpeed = stopOrbiter ;    //create global traction motor speed 
int tractionMotorSpeedOld = stopOrbiter;
int cycleCount = 0;                       //realtime think delay variable
volatile unsigned long position = 0;
boolean aliveLEDState = true;             //create a name for alive blinky light 
boolean realTimeRunStop = true;           //create a name for real time control loop flag
String command = "pauseHome";              //create a String object name for operator command string
unsigned long oldLoopTime = 0;            //create a name for past loop time in milliseconds
unsigned long newLoopTime = 0;            //create a name for new loop time in milliseconds
unsigned long cycleTime = 0;              //create a name for elapsed loop cycle time
const long controlLoopInterval = 1500 ;  //create a name for control loop cycle time in milliseconds

//================================================================================================
// Startup code to configure robot and pretest all robot functionality (to run once) 
// and code to setup robot mission for launch.
//================================================================================================
void setup() {
  // Robot control setup code here, to run once:
  //Serial.println(" Orbiter Robot Controller Starting Up! Watch your fingers! ");
  pinMode(aliveLED, OUTPUT);                 // initialize aliveLED pin as an output
  pinMode(soundPin, OUTPUT);                 // initialize soundPin pin as an output
  Serial.begin(9600);                        // start serial communications
  
  // Robot mission setup code here, to run once:
  dropServo.attach(5);                       //attach drop servo to pin 5
  tractionServo.attach(6);                   //attach traction motor to pin 6

  tone(soundPin,600,600);                     //Play start warning sound
  delay(600);
  tone(soundPin,200,300);                     //Play start warning sound
  delay(600);
  tone(soundPin,1000,600);                    //Play start warning sound
  delay(600);  
  
  delay(1000);
  dropServo.write(hookClosed);
  delay(1000);
  dropServo.write(hookOpen);
   delay(1000);
  tractionServo.write(moveDown);
   delay(1000);
  tractionServo.write(moveHome);
   delay(1000);
  tractionServo.write(stopOrbiter);
   delay(1000);

  attachInterrupt(digitalPinToInterrupt(2), Encoder_interrupt, CHANGE);
   
  tone(soundPin,1000,600);                    //Play start warning sound
  delay(600);
  tone(soundPin,200,300);                     //Play start warning sound
  delay(600);
  tone(soundPin,600,300);                     //Play start warning sound
  delay(600);  
  position = 0;                               //Reset the position once homed
}

//==================================================================================================
// Flight code to run continuously until robot is powered down 
//==================================================================================================
void loop() {
  
  // Outer Loop: Operator-Input-to-Robot and Robot-Reports-Back-State code in non-real-time
  // Inner Loop: Real-time dependant sense-think-act Robot-Orbiter control 
  
  // Outer loop: GET Operator Control Unit (OCU) Input: ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu-
  // OCU commands strings {stop, pauseHome, moveDownRange, dropRobot, pauseDownRange, returnHome}
  // Outer loop: GET Operator Control Unit (OCU) Input: ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu-
  //tractionServo.write(stopOrbiter);                           // stop orbiter while recieving input
  //delay(1000);                                  
  //command = getOperatorInput();                               // get operator input from serial monitor
  //if (command == "stop") realTimeRunStop = false;             // skip real time inner loop
  //else realTimeRunStop = true;                                // Set loop flag to run = true
    realTimeRunStop = true;                                     // Set loop flag to run = true
   
  //  Inner Loop: Fast Orbiter Control code in soft-real-time while loop structure below, to run repeatedly, 
  //  at a known fixed "real-time" periodic interval. This "soft real-time" loop timimg structure, runs 
  //  fast flight control code once every controlLoopInterval.
  // real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******
  // real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******  
  while(realTimeRunStop == true) {                            // if OCU-Stop not commanded, run control loop

    // Check if operator inputs a new command during real-time loop eecution 
    //if (Serial.available() > 0) {                             // check to see if operator typed at OCU
     // realTimeRunStop = false;                                // if OCU input typed, stop control loop
     // command = Serial.readString();                          // read command string to clear buffer
     // break;                                                  // break out of real-time loop
     // }                                                       
   // else { realTimeRunStop = true;}                           // if no operator input, run real-time loop
    
    // Real-Time clock control. Check to see if one control loop cycle has elapesed before running this 
    // control code
    newLoopTime = millis();                                    // get current Arduino time (50 days till wrap)
    if (newLoopTime - oldLoopTime >= controlLoopInterval) {    // if true run flight code inside big if structure 
      oldLoopTime = newLoopTime;                               // reset time stamp 
      blinkAliveLED();                                         // toggle blinky alive light
      tone(soundPin,beepSound,150);                            // Play cycle sound
      //Serial.println("    ");
      //Serial.print("real time command is = ");
      //Serial.println(command);
      //Serial.print("hook position = ");
      //if (dropServoAngle == hookOpen) {Serial.print("hook open");} 
      //else //Serial.print("hook closed"); 
      //Serial.print("\t traction Direction = ");
      //if (tractionMotorSpeed == moveDown) {Serial.println("moving down");}
      //if (tractionMotorSpeed == stopOrbiter) {Serial.println("orbiter stopped");}
      //if (tractionMotorSpeed == moveHome) {Serial.println("moving home"); } 
     
    // SENSE  sense---sense---sense---sense---sense---sense---sense---sense---sense---sense---sense--------   
    homeRange = analogRead(IRhomeRange); 
    downRange = analogRead(IRdownRange);
    Serial.print("homeRange = ");
    Serial.print(homeRange);
    Serial.print("\t downRange = ");
    Serial.println(downRange);
       
    // THINK  think---think---think---think---think---think---think---think---think---think---think---------
    // pick robot behavior based on operator input command typed at console
       if ( command == "stop"){
         //Serial.println("Stop Robot");
         
         realTimeRunStop = false;                               //exit real time control loop
         break;
       } 
       else if (command == "pauseHome"){                        //Wait at home to load rover
         command = Think_pauseHome (5);                         // wait 5 cycles
         //realTimeRunStop = true;                              //don't exit real time loop               
       }
       else if (command == "moveDownRange"){                    //Move robot down range till drop target
         command = Think_moveDownRange(downRange);
         //realTimeRunStop = true;                              //run loop continually 
       }
        else if (command == "dropRobot"){                       //Drop Rover
         command = Think_dropRobot ();                          
         //realTimeRunStop = true;                              //run loop continually                
       }
       else if (command == "pauseDownRange"){                   //Wait downrange to Drop Rover
         command = Think_pauseDownRange(5); 
         //realTimeRunStop = true;                              // don't exit realtime loop                      
       }
       else if (command == "returnHome"){                       //Return Home
         command = Think_returnHom(homeRange);
         //realTimeRunStop = true;                              //run loop continually                
       }
       else
       {
         //Serial.println("***** WARNING *******Invalid Input, Robot Stopped, Please try again!");
         realTimeRunStop = false;                                 //exit real time control loop
       } 
     
    // ACT    act---act---act---act---act---act---act---act---act---act---act---act---act---act------------
       dropServo.write(dropServoAngle);              // tell servo to go to position dropServoAngle
       tractionServo.write(tractionMotorSpeed);      // drive traction motor down stop or back
      /*int hook1;
      int track1;
       hook1 = driveHook(dropServoAngleOld,dropServoAngle);
       Serial.print(hook1);
       dropServoAngleOld = dropServoAngle;
       track1 = driveTractionMotor(tractionMotorSpeedOld, tractionMotorSpeed);
       tractionMotorSpeedOld = tractionMotorSpeed;
       Serial.print(track1);*/
       
       
    // Check to see if all code ran successfully in one real-time increment
       cycleTime = millis()-newLoopTime;                                 // calculate loop execution time   
       if( cycleTime > controlLoopInterval){
         //Serial.println("******************************************");
         //Serial.println("error - real time has failed, stop robot!");    // loop took too long to run
         //Serial.print(" 1000 ms real-time loop took = ");
         //Serial.println(cycleTime);                                      // print loop time
         //Serial.println("******************************************");
         break;                                                          // break out of real-time inner loop  
        }                   
    } // Inner Loop: end of  "if (newLoopTime - oldLoopTime >= controlLoopInterval)" real-time loop structure        
  } // Inner Loop: end of "inner"  "while(realTimeRunStop == true)" real-time control loop
  // End of real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******
  // End of real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******
   
  // SEND Robot State to Operator Control Unit (OCU)ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---------------
    // send robot status to operator  
    //Serial.println("=======================================================================================");
    //Serial.println("| Robot control loop stopping to wait for new command ");       
    
} // End of  "outer" void loop()
 // End of Outer loop: Operator Control Unit (OCU) Input: ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu-
 // End of Outer loop: Operator Control Unit (OCU) Input: ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu-
//===================================================================================================================
// END OF Robot Flight Code
//===================================================================================================================


//===================================================================================================================
//===================================================================================================================
// FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----  
// Functions for each section of above code 

//-------------------------------------------------------------------------------------------
// Functions for setup code 
//-------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------
// Functions for robot flight code
//-------------------------------------------------------------------------------------------

// Realtime loop functions loop---loop---loop---loop---loop---loop---loop---loop---loop---loop----

void blinkAliveLED(){
  // This function toggles state of aliveLED blinky light LED
  // if the LED is off turn it on and vice-versa:
    if (aliveLEDState == LOW) {
      aliveLEDState = HIGH;
    } else {
      aliveLEDState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(aliveLED, aliveLEDState); 
}

// OCU functions ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu------------

String getOperatorInput(){
  // This function prints operator command options on the serial console and prompts 
  // operator to input desired robot command
  Serial.println("    ");
  Serial.println("=======================================================================================");
  Serial.println("| Robot Behavior-Commands: pauseHome(load robot), stop(stops motors), moveDownRange |");
  Serial.println("| Robot Behavior-Commands: pauseDownRange, dropRobot, returnHome |");
  Serial.println("|                                                                                     |");
  //Serial.println("=======================================================================================");
  Serial.println("|   Please type desired robot behavior in command line at top of this window          |");   
  Serial.println("|    and then press SEND button.                                                      |");
  Serial.println("=======================================================================================");               
  while (Serial.available()==0) {};                             // do nothing until operator input typed
  command = Serial.readString();                                // read command string
  Serial.print("| New robot behavior command is:  ");           // give command feedback to operator
  Serial.println(command);
  Serial.println("| Type 'stop' to stop control loop and wait for new command                           |");
  Serial.println("=======================================================================================");
  return command;  
}

// SENSE functions sense---sense---sense---sense---sense---sense---sense---sense---sense---
// place sense functions here

void Encoder_interrupt(){
  if(tractionMotorSpeed > 90){
    position += 1;
  }
}

// THINK functions think---think---think---think---think---think---think---think---think---
// place think functions here

String Think_pauseHome (int cyclesToWait){            // waits at home for 5 cycles with hook open
  tractionMotorSpeed = stopOrbiter;                   // stop traction drive 0 =down 90=off 180=home
  cycleCount = cycleCount + 1;                        // increment cycles
  if(cycleCount > cyclesToWait){
  cycleCount = 0;                                     // reset count 
  position = 0;
  dropServoAngle = hookClosed;                        // close drop servo hook  
  return("moveDownRange");
  }
  else {
    beepSound = pauseSound;                            // Play cycle sound                                        
    dropServoAngle = hookOpen;                         //open drop servo hook 
    return("pauseHome");
    }
}

String Think_moveDownRange (int rangeToTarget){       // Moves orbiter down range with closed hook
  tractionMotorSpeed = moveDown;                      // run orbiter down range
  beepSound = runSound;  
  Serial.println(position);
  if (rangeToTarget > 250) {                          // drop rover when taregt gets in range 300
  return("dropRobot");
  }
  else {return("moveDownRange");} 
}

String Think_dropRobot (){                            // Drops rover from orbiter
  //Serial.println(" droppping rover ");
  tone(soundPin,600,300);                             //Play start warning sound
  delay(100);
  tone(soundPin,200,300);                             //Play start warning sound
  delay(100);
  tone(soundPin,1000,600);                            //Play start warning sound
  delay(100);  
  tractionMotorSpeed = moveDown;                      // run orbiter down range 
  dropServoAngle = hookOpen;                          //open drop servo hook  
  return("pauseDownRange"); 
}

String Think_pauseDownRange (int cyclesToWait){        // waits down range for 5 cycles with hook open
  tractionMotorSpeed = stopOrbiter;                            // stop traction drive 0 =down 90=off 180=home
  cycleCount = cycleCount + 1;                         // increment cycles
  if(cycleCount > cyclesToWait){
  cycleCount = 0;                                      // reset count  
  dropServoAngle = hookOpen;                           // open drop servo hook  
  return("returnHome");
  }
  else {
    beepSound = pauseSound;                             // Play cycle sound                                        
    dropServoAngle = hookOpen;                          //open drop servo hook  0=open 90=closed
    return("pauseDownRange");
    }
}

String Think_returnHom (int rangeToTarget){          // Moves orbiter home with open hook
  //Serial.println(" Move robot down range ");
  tractionMotorSpeed = moveHome;                      // run orbiter wn home 0 =down 90=off 180=home
  beepSound = returnSound;  
  dropServoAngle = hookOpen;                          //open drop servo hook  0=open 90=closed
  //Serial.println(rangeToTarget);
  if (rangeToTarget > 200) {                          // drop rover when taregt gets in range
  return("pauseHome");
  }
  else {return("returnHome");}
}

// ACT functions act---act---act---act---act---act---act---act---act---act---act---act---act---
// place ACT functions here


// END of Functions
//============================================================================================
// END OF All Robot Control CODE
