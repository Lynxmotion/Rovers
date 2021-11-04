// Libraries used in this example
  #include "ppm.h"
  #include <Servo.h> 

// Variable related to the control loop frequency
  long previousMillis = 0;        // will store last time LED was updated
  long interval = 10;           // interval at which to blink (milliseconds)

// 
  Servo FL_motor;
  Servo RL_motor;
  Servo FR_motor;
  Servo RR_motor;
  
  int motorFLspeed = 0;
  int motorRLspeed = 0;
  int motorFRspeed = 0;
  int motorRRspeed = 0;

// Motor direction modifier either 1 or -1 to change it
  int motorFLdirection = -1;
  int motorRLdirection = 1;
  int motorFRdirection = -1;
  int motorRRdirection = 1;

// PPM channel layout (update for your situation)
#define THROTTLE      3
#define RUDDER        4
#define PITCH         2
#define ROLL          1
#define SWITCH3WAY_1  5
#define BUTTON        6
#define SWITCH3WAY_2  7     // trim-pot for left/right motor mix  (face trim)
#define POT           8     // trim-pot on the (front left edge trim)

short throttle;
short rudder;
short pitch;
short roll;
short switch3way_1;
short button;
short switch3way_2;
short pot;

const int INPUT_DEADBAND = 10;             // consider channel at 0 until at least this much is sensed
const int OUTPUT_DEADBAND = 20;

int mode = 0;



void setup(){ 
  // Buffer between USB & ATmega for LSS-2IO
  pinMode(7, OUTPUT); 
  digitalWrite(7, LOW);

  // Pin mode for the 4 motor outputs
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  
  // Creation of the "servo" motors
  FL_motor.attach(3);
  RL_motor.attach(5);
  FR_motor.attach(6);
  RR_motor.attach(9);

  // Start the PPM function
  ppm.begin(A0, false);

  Serial.begin(115200);

}

void loop(){  
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) { 
    previousMillis = currentMillis;   
    PPMupdate();
    remoteCommand();
    switch (mode){
      case 0:
        moveIdle();
        break;
      case 1:
        moveMix();
        break;
    }
    debug();
    MOTORupdate();
  }
  
}

void PPMupdate(){ 
  throttle        =     constrain(ppm.read_midstick(THROTTLE), -500, 500);
  roll            =   - constrain(ppm.read_midstick(ROLL), -500, 500);
  pitch           =     constrain(ppm.read_midstick(PITCH), -500, 500);
  rudder          =   - constrain(ppm.read_midstick(RUDDER), -500, 500);
  switch3way_1    =     ppm.read_channel(SWITCH3WAY_1);
  button          =     ppm.read_channel(BUTTON);
  switch3way_2    =     ppm.read_channel(SWITCH3WAY_2);
  pot             =     ppm.read_channel(POT);
}

void remoteCommand(){ 
  if ((switch3way_1 >= 1400) && (switch3way_1 <= 1600))
    mode = 0;
  else{
    mode = 1;
  }
}

void MOTORupdate(){ 
  FL_motor.writeMicroseconds(motorFLspeed);
  RL_motor.writeMicroseconds(motorRLspeed);
  FR_motor.writeMicroseconds(motorFRspeed);
  RR_motor.writeMicroseconds(motorRRspeed);
}

void moveIdle(){ 
  motorFLspeed = 1500;
  motorRLspeed = 1500;
  motorFRspeed = 1500;
  motorRRspeed = 1500;
}

void moveMix(){
  motorFLspeed = 1500 + (clamp(-500, (pitch + roll + rudder), 500) * motorFLdirection);
  motorRLspeed = 1500 + (clamp(-500, (pitch - roll + rudder), 500) * motorRLdirection);
  motorFRspeed = 1500 + (clamp(-500, (pitch - roll - rudder), 500) * motorFRdirection);
  motorRRspeed = 1500 + (clamp(-500, (pitch + roll - rudder), 500) * motorRRdirection);

  motorFLspeed = deadband(motorFLspeed, OUTPUT_DEADBAND);
  motorRLspeed = deadband(motorRLspeed, OUTPUT_DEADBAND);
  motorFRspeed = deadband(motorFRspeed, OUTPUT_DEADBAND);
  motorRRspeed = deadband(motorRRspeed, OUTPUT_DEADBAND);
}

void moveForwardReverse(){ 
  motorFLspeed = deadband((map(pitch, -100, 100, 2000, 1000)), OUTPUT_DEADBAND);
  motorRLspeed = deadband((map(pitch, -100, 100, 1000, 2000)), OUTPUT_DEADBAND);
  motorFRspeed = deadband((map(pitch, -100, 100, 2000, 1000)), OUTPUT_DEADBAND);
  motorRRspeed = deadband((map(pitch, -100, 100, 1000, 2000)), OUTPUT_DEADBAND);
}

void moveLeftRight(){ 
  motorFLspeed = deadband((map(roll, -100, 100, 1000, 2000)), OUTPUT_DEADBAND);
  motorRLspeed = deadband((map(roll, -100, 100, 1000, 2000)), OUTPUT_DEADBAND);
  motorFRspeed = deadband((map(roll, -100, 100, 2000, 1000)), OUTPUT_DEADBAND);
  motorRRspeed = deadband((map(roll, -100, 100, 2000, 1000)), OUTPUT_DEADBAND);
}

void moveCircle(){ 
  motorFLspeed = deadband((map(rudder, -100, 100, 1000, 2000)), OUTPUT_DEADBAND);
  motorRLspeed = deadband((map(rudder, -100, 100, 2000, 1000)), OUTPUT_DEADBAND);
  motorFRspeed = deadband((map(rudder, -100, 100, 2000, 1000)), OUTPUT_DEADBAND);
  motorRRspeed = deadband((map(rudder, -100, 100, 1000, 2000)), OUTPUT_DEADBAND);
}

void debug(){  
//    // PPM values for the Arduino Serial Plotter
//    Serial.print("Throttle:");        Serial.print(throttle);       Serial.print(" ");
//    Serial.print("Roll:");            Serial.print(roll);           Serial.print(" ");
//    Serial.print("Pitch:");           Serial.print(pitch);          Serial.print(" ");
//    Serial.print("Rudder:");          Serial.print(rudder);         Serial.print(" ");
//    Serial.print("Switch_3way_1:");   Serial.print(switch3way_1);   Serial.print(" ");
//    Serial.print("Button:");          Serial.print(button);         Serial.print(" ");
//    Serial.print("Switch_3way_2:");   Serial.print(switch3way_2);   Serial.print(" ");
//    Serial.print("Pot:");             Serial.print(pot);            Serial.print(" ");
//    Serial.println();

    // Motor output values for the Arduino Serial Plotter
//    Serial.print("motorFLspeed:");     Serial.print(motorFLspeed);       Serial.print(" ");
//    Serial.print("motorRLspeed:");     Serial.print(motorRLspeed);       Serial.print(" ");
//    Serial.print("motorFRspeed:");     Serial.print(motorFRspeed);       Serial.print(" ");
//    Serial.print("motorRRspeed:");     Serial.print(motorRRspeed);       Serial.print(" ");
//    Serial.println();

}
