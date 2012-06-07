//Automous Rover code
//Written by Devon Simmons
//Hardware : Arduino & Sabertooth 2x10 RC

#include <Servo.h>
#include <EEPROM.h>

#define Debug

#define Wheels_Left 12
#define Wheels_Right 13
#define ir_Left_pin A1
#define ir_Right_pin A2
#define ir_Rear_pin A3

int ir_Left;
int ir_Right;
int ir_Rear;

Servo Left;
Servo Right;

int LeftSpeed = 1500;
int RightSpeed = 1500;

byte Offsets[2] = {127, 127};

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif

  Left.attach(Wheels_Left);
  Right.attach(Wheels_Right);
  Left.write(90);
  Right.write(90);
  
  delay(100);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  if(!digitalRead(7))
    Config_Offsets();
  else
    Load_Offsets();
}

void Load_Offsets()
{
  if(EEPROM.read(0) != 0)
    return;
  else
  {
    Offsets[0] = EEPROM.read(1);
    Offsets[1] = EEPROM.read(2);
  }
}

void Config_Offsets()
{
  tone(5, 1000, 1000); 
  delay(2000);
  
  for(int x = 0; x < 2; x++)
  {
    while(digitalRead(7))
    {
      if(digitalRead(8) == LOW)
      {
        Offsets[x] = Offsets[x] - 1;
        delay(100);
      }
      else if(digitalRead(9) == LOW)
      {
        Offsets[x] = Offsets[x] + 1;
        delay(100);
      }
      if(x)
        Left.write(90 + Offsets[x]);
      else
        Right.write(90 + Offsets[x]);
    }
    delay(500); 
  }
  EEPROM.write(0, 0);
  EEPROM.write(1, Offsets[0]);
  EEPROM.write(2, Offsets[1]);
  for(int x = 0; x < 5; x++)
  {
    tone(5, 100*x + 500, 100);
    delay(100);
  }
}

void readIr()
{
  for(int x = 0; x < 10; x++)
  {
    ir_Left += analogRead(ir_Left_pin);
    ir_Right += analogRead(ir_Right_pin);
    ir_Rear += analogRead(ir_Rear_pin);
  }
  ir_Left /= 85;
  ir_Right /= 85;
  ir_Rear /= 85;
#ifdef DEBUG
  Serial.print(ir_Left);
  Serial.print(" ");
  Serial.print(ir_Right);
  Serial.print(" ");
  Serial.println(ir_Rear);
#endif
}

void loop()
{
  readIr();
  
  LeftSpeed = max(LeftSpeed - 10, 1250);
  RightSpeed = max(RightSpeed - 10, 1250);
  
  LeftSpeed = min(LeftSpeed + ir_Left, 1750);
  RightSpeed = min(RightSpeed + ir_Right, 1750);
  
  if(ir_Rear > 15)
  {
    LeftSpeed = max(LeftSpeed - ir_Rear, 1250);
    RightSpeed = max(RightSpeed - ir_Rear, 1250);
  }
  
  Left.writeMicroseconds(LeftSpeed);
  Right.writeMicroseconds(RightSpeed);
  
  delay(50);
}
