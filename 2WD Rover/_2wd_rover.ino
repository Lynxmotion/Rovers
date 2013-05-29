//*********************************************************************************
//  www.Lynxmotion.com
//  Basic code for 2WD rover using continuous rotation servos, controlled via PS2
// Right now, the library does NOT support hot pluggable controllers, meaning 
// you must always either restart your Arduino after you conect the controller, 
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
 // Setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
 // If the red LED on the receiver does not become solid when you turn on the PS2 handheld unit, it's likely because the pin selection above is incorrect.
 // Troubleshooting code here was removed - can be seen in PS2 sample code
}


void loop(){
  
  ps2x.read_gamepad();          //read controller 

    // Creates a dead zone so the rover will 
    if(ps2x.Analog(PSAB_PAD_UP) < Deadzone && ps2x.Analog(PSAB_PAD_DOWN) < Deadzone && ps2x.Analog(PSAB_PAD_RIGHT) < Deadzone && ps2x.Analog(PSAB_PAD_LEFT) < Deadzone){  
    servoleft.write(90); // servo signal is between 0 and 180 degrees
    servoright.write(90);
    }
         
    else {
  
      // This code simply has the robot move forward / reverse, and rotate clockwise and counter-clockwise. 
      // Feel free to include proportional control, add sensors etc.
      
      if(ps2x.Button(PSB_PAD_UP)) {         //will be TRUE as long as button is pressed
       servoleft.write(40);
       servoright.write(140);
      }
      if(ps2x.Button(PSB_PAD_RIGHT)){
       servoleft.write(70);
       servoright.write(70);
      }
      if(ps2x.Button(PSB_PAD_LEFT)){
       servoleft.write(110);
       servoright.write(110);
      }
      if(ps2x.Button(PSB_PAD_DOWN)){
       servoleft.write(140);
       servoright.write(40);
      }   
}
 delay(50);
     
}
