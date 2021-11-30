// Libraries used in this example
  #include "ppm.h"
  #include <Servo.h> 

// Variable related to the control loop frequency
  long previousMillis = 0;  // will store last time LED was updated
  long interval = 10;       // interval at which to blink (milliseconds)

// Creation of the Servo Library Objects
  Servo CH1_motor;
  Servo CH2_motor;

// Variables for the PINs
  int CH1_pin = 3;
  int CH2_pin = 5;

// Variables to store the speed values that will be sent for each motors
  int motorCH1speed = 0;
  int motorCH2speed = 0;

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

// Enabling Movement Mode
int mode = 0;

void setup(){ 
  // Buffer between USB & ATmega for LSS-2IO
  pinMode(7, OUTPUT); 
  digitalWrite(7, LOW);

  // Pin mode for the 4 motor outputs
  pinMode(CH1_pin, OUTPUT);
  pinMode(CH2_pin, OUTPUT);
  
  // Creation of the "servo" motors
  CH1_motor.attach(CH1_pin);
  CH2_motor.attach(CH2_pin);

  // Start the PPM function
  ppm.begin(A0, false);

  Serial.begin(115200);

}

void loop(){  
  unsigned long currentMillis = millis();
  
  // Do this at every interval time
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
  // To reverse the direction of the signal, add a "-" in front of the constrain
  throttle        =   constrain(ppm.read_midstick(THROTTLE),      -500, 500);
  roll            =   constrain(ppm.read_midstick(ROLL),          -500, 500);
  pitch           = - constrain(ppm.read_midstick(PITCH),         -500, 500);
  rudder          =   constrain(ppm.read_midstick(RUDDER),        -500, 500);
  switch3way_1    =   constrain(ppm.read_midstick(SWITCH3WAY_1),  -500, 500);
  button          =   constrain(ppm.read_midstick(BUTTON),        -500, 500);
  switch3way_2    =   constrain(ppm.read_midstick(SWITCH3WAY_2),  -500, 500);
  pot             =   constrain(ppm.read_midstick(POT),           -500, 500);
}

void remoteCommand(){ 
  if ((switch3way_1 >= -100) && (switch3way_1 <= 100))
    mode = 0;
  else{
    mode = 1;
  }
}

void moveIdle(){ 
  motorCH1speed = 1500;
  motorCH2speed = 1500;
}

void moveMix(){
  motorCH1speed  = map(pitch, -500, 500, 1000, 2000);
  motorCH1speed = deadband(motorCH1speed, OUTPUT_DEADBAND);
  
  motorCH2speed  = map(roll, -500, 500, 1000, 2000);
  motorCH2speed = deadband(motorCH2speed, OUTPUT_DEADBAND);
}

void MOTORupdate(){ 
  CH1_motor.writeMicroseconds(motorCH1speed);
  CH2_motor.writeMicroseconds(motorCH2speed);
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

//    // Motor Outputs for the Arduino Serial Plotter
//    Serial.print("motorCH1speed:");     Serial.print(motorCH1speed);       Serial.print(" ");
//    Serial.print("motorCH2speed:");     Serial.print(motorCH2speed);       Serial.print(" ");
//    Serial.println();
}
