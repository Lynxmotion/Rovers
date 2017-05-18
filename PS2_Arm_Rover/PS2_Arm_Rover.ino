#include <Servo.h>
#include <PS2X_lib.h>
#include <EEPROM.h>

// Uncomment to obtain debug information in serial monitor
//#define DEBUG
#define DEBUG_BAUD  9600

// Comment to disable the Force Sensitive Resister on the gripper
//#define FSRG

// FSRG pin Must be analog!!
#define FSRG_pin A1

// Select which arm by uncommenting the corresponding line
//#define AL5A
//#define AL5B
#define AL5D

// Uncomment for digital servos in the Shoulder and Elbow
// that use a range of 900ms to 2100ms
//#define DIGITAL_RANGE

#ifdef AL5A
const float A = 3.75;
const float B = 4.25;
#elif defined AL5B
const float A = 4.75;
const float B = 5.00;
#elif defined AL5D
const float A = 5.75;
const float B = 7.375;
#endif

// PS2 input pins
#define PS2_DAT 8    // Some assembly instructions may have DAT & ATT reversed. If your PS2 receiver is not found in debug, try reversing the pin numbers.
#define PS2_CMD 7
#define PS2_ATT 6    // Some assembly instructions may have DAT & ATT reversed. If your PS2 receiver is not found in debug, try reversing the pin numbers.
#define PS2_CLK 9

// PS2 analog joystick Deadzone
#define Deadzone 4

// Arm Servo pins
#define Base_pin 2
#define Shoulder_pin 3
#define Elbow_pin 4
#define Wrist_pin 10
#define Gripper_pin 11
#define WristR_pin 12

// Rover motor control mode
#define ROVER_DIFFERENTIAL    // Mixed mode is used. DIP switch #1 is ON on the Sabertooth controller. Comment this line to go into independant mode (switch #1 OFF).

// Rover Servo pins
#define Throttle_pin A2      // In independant mode, throttle is used as the LEFT motor channel. Reverse pin numbers if going backwards.
#define Steering_pin A3      // In independant mode, steering is used as the RIGHT motor channel. Reverse pin numbers if going backwards.

// Onboard Speaker
#define Speaker_pin 5

// Radians to Degrees constant
const float rtod = 57.295779;

// Arm Speed Variables
float Speed = 1.0;
int sps = 3;

// Rover Speed Variables
float RSpeed = 0.8;
int Rsps = 3;

// Servo Objects
Servo Elb;
Servo Shldr;
Servo Wrist;
Servo Base;
Servo WristR;
Servo Gripper;
Servo Throttle;      // In independant mode, used as the left channel
Servo Steering;      // In independant mode, used as the right channel

// Arm Current Pos
float X = 4;
float Y = 4;
int Z = 90;
int G = 90;
int WR = 90;
float WA = 0;

// Arm temp pos
float tmpx = 4;
float tmpy = 4;
int tmpz = 90;
int tmpg = 90;
int tmpwr = 90;
float tmpwa = 0;

// PS2X Variables
PS2X ps2x;
int error = 0; 
byte type = 0;

// Offsets
byte Offsets[2] = {127, 127};

boolean mode = true;

void setup()
{
  // Attach motor controller pins
  Throttle.attach(Throttle_pin);
  Steering.attach(Steering_pin);
  
  // Set default movement to IDLE
  Throttle.writeMicroseconds(1500);
  Steering.writeMicroseconds(1500);
  
  pinMode(PS2_CMD, INPUT);
  if(!digitalRead(PS2_CMD))
  {
    tone(Speaker_pin, 1000, 500);
    delay(1000);
    pinMode(PS2_DAT, INPUT);
    pinMode(PS2_CLK, INPUT);
    for(int i=1; i>-1; i--)
    {
      while(digitalRead(PS2_CMD))
      {
        if(!digitalRead(PS2_DAT))
        {
          Offsets[i] = min(Offsets[i] + 1, 227);
          i? Steering.writeMicroseconds(1500 - 127 + Offsets[i]) : Throttle.writeMicroseconds(1500 - 127 + Offsets[i]);
        }
        else if(!digitalRead(PS2_CLK))
        {
          Offsets[i] = max(Offsets[i] - 1, 27);
          i? Steering.writeMicroseconds(1500 - 127 + Offsets[i]) : Throttle.writeMicroseconds(1500 - 127 + Offsets[i]);
        }
        delay(50);
      }
      delay(500);
    }
    EEPROM.write(0, 0);
    EEPROM.write(1, Offsets[0]);
    EEPROM.write(2, Offsets[1]);
    tone(Speaker_pin, 3000, 1000);
  }
  else
  {
    if(EEPROM.read(0) != 0);
    else
    {
      Offsets[0] = EEPROM.read(1);
      Offsets[1] = EEPROM.read(2);
    }
  }

#ifdef DEBUG
  Serial.begin(DEBUG_BAUD);
#endif

  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  
#ifdef DEBUG
  if(error == 0)
    Serial.println("Found Controller, configured successful");
   
  else if(error == 1)
    Serial.println("No controller found");
   
  else if(error == 2)
    Serial.println("not accepting commands");
   
  else if(error == 3)
    Serial.println("refusing Pressures mode");
   
  type = ps2x.readType(); 
    switch(type) 
    {
      case 0:
       Serial.println("Unknown");
       break;
      case 1:
       Serial.println("DualShock");
       break;
      case 2:
       Serial.println("GuitarHero");
       break;
    }
#endif

  // Attach arm servo pins
  Base.attach(Base_pin);
  Shldr.attach(Shoulder_pin);
  Elb.attach(Elbow_pin);
  Wrist.attach(Wrist_pin);
  Gripper.attach(Gripper_pin);
  WristR.attach(WristR_pin);
}

// Will return 1 if move will result in an overflow or crash
int Arm(float x, float y, float z, int g, float wa, int wr) //Here's all the Inverse Kinematics to control the arm
{
  float M = sqrt((y*y)+(x*x));
  if(M <= 0)
    return 1;
  float A1 = atan(y/x);
  if(x <= 0)
    return 1;
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
  float Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
  float Shoulder = A1 + A2;
  Elbow = Elbow * rtod;
  Shoulder = Shoulder * rtod;
  if((int)Elbow <= 0 || (int)Shoulder <= 0)
    return 1;
  float Wris = abs(wa - Elbow - Shoulder) - 90;
#ifdef DIGITAL_RANGE
  Elb.writeMicroseconds(map(180 - Elbow, 0, 180, 900, 2100  ));
  Shldr.writeMicroseconds(map(Shoulder, 0, 180, 900, 2100));
#else
  Elb.write(180 - Elbow);
  Shldr.write(Shoulder);
#endif
  Wrist.write(180 - Wris);
  Base.write(z);
  WristR.write(wr);
  #ifndef FSRG
  Gripper.write(g);
  #endif
  Y = tmpy;
  X = tmpx;
  Z = tmpz;
  WA = tmpwa;
  #ifndef FSRG
  G = tmpg;
  #endif
  WR = tmpwr;
  return 0; 
}

void loop()
{
  ps2x.read_gamepad(); //update the ps2 controller
  
  int LSY = 128 - ps2x.Analog(PSS_LY);
  int LSX = ps2x.Analog(PSS_LX) - 128;
  int RSY = 128 - ps2x.Analog(PSS_RY);
  int RSX = ps2x.Analog(PSS_RX) - 128;
  
  #ifdef DEBUG
    Serial.print("PS2 Sticks: LX: ");
    Serial.print(PSS_LX);
    Serial.print(", LY = ");
    Serial.print(PSS_LY);
    Serial.print(", RX = ");
    Serial.print(PSS_RX);
    Serial.print(", RY = ");
    Serial.println(PSS_RY);
    Serial.print("PS2 Sticks: LSX: ");
    Serial.print(LSX);
    Serial.print(", LSY = ");
    Serial.print(LSY);
    Serial.print(", RSX = ");
    Serial.print(RSX);
    Serial.print(", RSY = ");
    Serial.println(RSY);
    Serial.println("");
    delay(100);
  #endif

  /* // Mode selection is inactive
  if(ps2x.ButtonPressed(PSB_SELECT))
  {
    mode = !mode;
    if(!mode)
    {
      Throttle.writeMicroseconds(1500 - 127 + Offsets[0]);
      Steering.writeMicroseconds(1500 - 127 + Offsets[1]);
    }
  }
  */
  
  /*  // Mode selection is inactive
  if(mode)
  {
  */
    // Rover wheel control
    if(ps2x.Button(PSB_RED))
    {
      // Turn right
      #ifdef DEBUG
        Serial.println("Rover turning right");
      #endif
      
      #ifdef ROVER_DIFFERENTIAL
        Throttle.write(140);
        Steering.write(40);
      #else
        Throttle.write(140);
        Steering.write(140);
      #endif
    }
    else if(ps2x.Button(PSB_PINK))
    {
      // Turn left
      #ifdef DEBUG
        Serial.println("Rover turning left");
      #endif
      
      #ifdef ROVER_DIFFERENTIAL
        Throttle.write(140);
        Steering.write(140);
      #else
        Throttle.write(40);
        Steering.write(40);
      #endif
    }
    else if(ps2x.Button(PSB_BLUE))
    {
      // Move in reverse
      #ifdef DEBUG
        Serial.println("Rover moving in reverse");
      #endif
      
      #ifdef ROVER_DIFFERENTIAL
        Throttle.write(40);
        Steering.write(90);
      #else
        Throttle.write(40);
        Steering.write(140);
      #endif
    }
    else if(ps2x.Button(PSB_GREEN))
    {
      // Move forward
      #ifdef DEBUG
        Serial.println("Rover moving forward");
      #endif
      
      #ifdef ROVER_DIFFERENTIAL
        Throttle.write(140);
        Steering.write(90);
      #else
        Throttle.write(140);
        Steering.write(40);
      #endif
    }   
    else
    {
      // Idle
      #ifdef DEBUG
        Serial.println("Rover idle");
      #endif
      
      #ifdef ROVER_DIFFERENTIAL
        Throttle.write(90);    // Adjust these values if the servos still move slightly
        Steering.write(90);
      #else
        Throttle.write(90);    // Adjust these values if the servos still move slightly
        Steering.write(90);
      #endif
    }
    
    /* // Control using right analog stick & mode is inactive
    if(RSY > Deadzone || RSY < -Deadzone)
      Throttle.writeMicroseconds(3000 - (RSY / 127.0 * 250 * RSpeed + 1500 - 127 + Offsets[0]));
    else
      Throttle.writeMicroseconds(1500 - 127 + Offsets[0]);
    if(RSX > Deadzone || RSX < -Deadzone)
      Steering.writeMicroseconds(3000 - (RSX / 127.0 * 250 * RSpeed + 1500 - 127 + Offsets[1]));
    else
      Steering.writeMicroseconds(1500 - 127 + Offsets[1]);
    
    if(ps2x.ButtonPressed(PSB_PAD_RIGHT))
      Rsps = min(Rsps + 1, 5);
    else if(ps2x.ButtonPressed(PSB_PAD_LEFT))
      Rsps = max(Rsps - 1, 1);
      
      RSpeed = Rsps*0.2 + 0.4;
    */
    
  /*  // Mode selection is inactive
  }
  else
  {
  */
    // Robotic arm movement control
    Serial.println("");
    Serial.print("Arm debug: ");
    if(RSY > Deadzone || RSY < -Deadzone)
    {
      tmpy = max(Y + RSY/1000.0*Speed, -5);

      #ifdef DEBUG
        Serial.print(", tmpy = ");
        Serial.print(tmpy);
      #endif
    }
  
    if(RSX > Deadzone || RSX < -Deadzone)
    {
      tmpx = max(X + RSX/1000.0*Speed, 0.001);

      #ifdef DEBUG
        Serial.print(", tmpx = ");
        Serial.print(tmpx);
      #endif
    }
  
    if(LSY > Deadzone || LSY < -Deadzone)
    {
      tmpwa = constrain(WA + LSY/100.0*Speed, 0, 180);

      #ifdef DEBUG
        Serial.print(", tmpwa = ");
        Serial.print(tmpwa);
      #endif
    }
  
    if(LSX > Deadzone || LSX < -Deadzone)
    {
      tmpz = constrain(Z + LSX/100.0*Speed, 0, 180);

      #ifdef DEBUG
        Serial.print(", tmpz = ");
        Serial.print(tmpz);
      #endif
    }
    
    if(ps2x.Button(PSB_R1))
    {
      #ifdef FSRG
      while(analogRead(FSRG_pin) < 400)
      {
        Gripper.write(min(Gripper.read() + 2, 170));
        if(Gripper.read() == 170)
          break;
        #ifdef DEBUG
        Serial.print(analogRead(FSRG_pin));
        Serial.print(" ");
        Serial.println(Gripper.read());
        #endif
        delay(10);
      }
      #else
      tmpg = min(G + 5*Speed, 170);
      #endif

      #ifdef DEBUG
        Serial.print(", tmpg = ");
        Serial.print(tmpg);
      #endif
    }
    if(ps2x.Button(PSB_R2))
    {
      #ifdef FSRG
      while(Gripper.read() > 90)
      {
        Gripper.write(max(Gripper.read() - 2, 90));
        #ifdef DEBUG
        Serial.println(Gripper.read());
        #endif
        delay(10);
      }
      #else
      tmpg = max(G - 5*Speed, 10);
      #endif

      #ifdef DEBUG
        Serial.print(", tmpg = ");
        Serial.print(tmpg);
      #endif
    }
     
    if(ps2x.Button(PSB_L1))
    {
      tmpwr = max(WR + 2*Speed, 0);

      #ifdef DEBUG
        Serial.print(", tmpwr = ");
        Serial.print(tmpwr);
      #endif
    }
    else if(ps2x.Button(PSB_L2))
    {
      tmpwr = min(WR - 2*Speed, 180);
      
      #ifdef DEBUG
        Serial.print(", tmpwr = ");
        Serial.print(tmpwr);
      #endif
    }
    
    if(ps2x.ButtonPressed(PSB_PAD_UP))
    {
      sps = min(sps + 1, 5);
      tone(Speaker_pin, sps*500, 200);
      
      #ifdef DEBUG
        Serial.print(", sps = ");
        Serial.print(sps);
      #endif
    }
    else if(ps2x.ButtonPressed(PSB_PAD_DOWN))
    {
      sps = max(sps - 1, 1);
      tone(Speaker_pin, sps*500, 200);
      
      #ifdef DEBUG
        Serial.print(", sps = ");
        Serial.print(sps);
      #endif
    }
    #ifdef DEBUG
      Serial.println("");
      Serial.println("");
    #endif
    
    Speed = sps*0.20 + 0.60;
          
    if(Arm(tmpx, tmpy, tmpz, tmpg, tmpwa, tmpwr))
    {
      #ifdef DEBUG
      Serial.print("NONREAL Answer");
      #endif
    }
    
    if(tmpx < 2 && tmpy < 2 && RSX < 0)
    {
      tmpy = tmpy + 0.05;
      Arm(tmpx, tmpy, tmpz, tmpg, tmpwa, tmpwr);
    }
    else if(tmpx < 1 && tmpy < 2 && RSY < 0)
    {
      tmpx = tmpx + 0.05;
      Arm(tmpx, tmpy, tmpz, tmpg, tmpwa, tmpwr);
    }
    /*  // Mode selection is inactive
    }
    */
  delay(30);
}
