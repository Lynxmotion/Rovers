// Libraries used in this example
  #include <Servo.h> 

// Creation of the Servo Library Objects
  Servo LF_Motor;
  Servo LR_Motor;
  Servo RF_Motor;
  Servo RR_Motor;
  
// Variables for the PINs
  int LF_Pin = 3;
  int LR_Pin = 5;
  int RF_Pin = 6;
  int RR_Pin = 9;

// Variables for the different signals
  int mainSpeed = 25;
  int LF_Signal = 0;
  int LR_Signal = 0;
  int RF_Signal = 0;
  int RR_Signal = 0;

// Variable for the serial reading
  char keyboardVal;

void setup(void){
// Set the digital PINs as OUTPUTs
  pinMode(LF_Pin, OUTPUT);
  pinMode(LR_Pin, OUTPUT);
  pinMode(RF_Pin, OUTPUT);
  pinMode(RR_Pin, OUTPUT);

// Attach the PINs to the corresponding Objects
  LF_Motor.attach(LF_Pin);
  LR_Motor.attach(LR_Pin);
  RF_Motor.attach(RF_Pin);
  RR_Motor.attach(RR_Pin);

// Start a Serial port for the Controls
  Serial.begin(115200);
}

void loop(void){
// Keyboard Capture & Motor Mixing
  keyboardCapture();
// Mixing the Motors OUTPUTs
  outputMix();
// Updating the Outputs
  outputsUpdate();
}

void keyboardCapture(){
  while (Serial.available() < 1) {} // Wait until a character is received
  keyboardVal = Serial.read();
}

void outputMix(){
  switch(keyboardVal){ 
    case 'w':                   //Move Forward
      LF_Signal = mainSpeed;
      LR_Signal = -mainSpeed;
      RF_Signal = mainSpeed;
      RR_Signal = -mainSpeed;
      break;
    case 's':                   //Move Backwards
      LF_Signal = -mainSpeed;
      LR_Signal = mainSpeed;
      RF_Signal = -mainSpeed;
      RR_Signal = mainSpeed;
      break;
    case 'a':                   //Turn Left
      LF_Signal = -mainSpeed;
      LR_Signal = mainSpeed;
      RF_Signal = mainSpeed;
      RR_Signal = -mainSpeed;
      break;
    case 'd':                   //Turn Right
      LF_Signal = mainSpeed;
      LR_Signal = -mainSpeed;
      RF_Signal = -mainSpeed;
      RR_Signal = mainSpeed;
      break;
    default:                    //Anything else Stop
      LF_Signal = 0;
      LR_Signal = 0;
      RF_Signal = 0;
      RR_Signal = 0;
      break;
  }
}

void outputsUpdate(void){
  LF_Motor.writeMicroseconds(map(LF_Signal, 100, -100, 2000, 1000));
  LR_Motor.writeMicroseconds(map(LR_Signal, 100, -100, 2000, 1000));
  RF_Motor.writeMicroseconds(map(RF_Signal, 100, -100, 2000, 1000));
  RR_Motor.writeMicroseconds(map(RR_Signal, 100, -100, 2000, 1000));
}
