//*********************************************************************************
// www.Lynxmotion.com
// Basic code for 2WD rover using continuous rotation servos, controlled via PS2
// Right now, the library does NOT support hot pluggable controllers, meaning 
// you must always either restart your Arduino after you connect the controller, 
// or call config_gamepad(pins) again after connecting the controller.
//*********************************************************************************

#include <PS2X_lib.h>  //for v1.6
#include <Servo.h> 

// create PS2 Controller Class
PS2X ps2x; 
int error = 0; 
byte vibrate = 0;

// create servo objects to control the servos
Servo servoleft; // left servo
Servo servoright; // right servo

#define Deadzone 10 //PS2 analog joystick Deadzone

void setup(){
 Serial.begin(9600);

 // Attaches servos to digital pins 3 and 4 on the BotBoarduino
 servoleft.attach(3);
 servoright.attach(4); 
 
 pinMode(7, INPUT);
 
 error = ps2x.config_gamepad(9,7,6,8, true, true);   
}


void loop(){
  
  ps2x.read_gamepad();          //read controller 

      // This code uses the colored buttons on the right side of the joystick 
      
      if(ps2x.Button(PSB_RED)) {
       servoleft.write(70);
       servoright.write(70);

      }
      else if(ps2x.Button(PSB_PINK)){
       servoleft.write(110);
       servoright.write(110);

      }
      else if(ps2x.Button(PSB_BLUE)) {
       servoleft.write(140);
       servoright.write(40);
      }
      else if(ps2x.Button(PSB_GREEN)){
       servoleft.write(40);
       servoright.write(140);

      }   
      else {
       servoleft.write(90);    // Adjust these values if the servos still move slightly
       servoright.write(90);
      }
 delay(50);
     
}

