;Project Lynxmotion Rover including Robot Arm
;Description: Rover with 6DOF Robot Arm, controlled by a PS2 remote
;Software version: V1.2 beta
;Date: 18-05-2009
;Programmer: Jeroen Janssen (aka Xan)
;
;Hardware setup: BB2 with ATOM 28 Pro, PS2 remote (See further down for connections)
;
;NEW IN V1.2 beta
;	- New Arm initialization
;	- Overflow solved 		- 		Kurt Eckhardt (aka Kurte)
;	- Maximal movements limited to overstretch the arm
;
;PS2 CONTROLS:
;	- Start				Turn on/off Rover / Arm
;	- select			Switch between Rover and Arm control
;
;	ROVER CONTROL
;	Tank mode:
;	- Left Stick U/D	Left Wheels forward / reverse
;	- Right Stick U/D	Right Wheels forward / reverse
;
;	One Stick mode:
;	- Right Stick U/D	;forward / reverse
;	- Right Stick L/R	;steering
;
;	universal:
;	- D-Pad up			Shift up gear (1 to 4)
;	- D-Pad down		Shift down gear (1 to 4)
;	- D-Pad left		N/A
;	- D-Pad right		N/A
;	- D-Pad left		N/A
;	- D-Pad right		N/A
;	- Triangle			N/A
;	- Circle			N/A
;   - Square        	N/A
;	- Cross				N/A
;	- L1				N/A
;	- L2				N/A
;	- L3				N/A
;	- R1				N/A
;	- R2				N/A
;	- R3				Change steering mode (Tank / One stick)
;
;	ARM CONTROL
;	- Left Stick U/D	Gripper Up/Down
;	- Left Stick L/R	Gripper Away/Back
;	- Right Stick U/D	Gripper Angle Up/Down
;	- Right Stick L/R	Base Left/Right
;	- D-Pad up			N/A
;	- D-Pad down		N/A
;	- D-Pad left		N/A
;	- D-Pad right		N/A
;	- Triangle			Return Wrist Rotate to the middle position
;	- Circle			N/A
;   - Square        	N/A
;	- Cross				Fully open Gripper
;	- L1				Gripper Close
;	- L2				Gripper Open
;	- L3				N/A
;	- R1				Wrist Rotate CW
;	- R2				Wrist Rotate CCW
;	- R3				N/A
;
;TODO:
;	- Movement to fixed point
;
;KNOWN BUGS:
;	- None
;
;
;====================================================================
;[SETUP]
;disable(0)/enable(1) the FSR Gripper 
EnableFSR con 0	

;Select Arm type
;NOTE: Enable only one arm!
;AL5A con 1
AL5B con 1
;AL5C con 1
;AL5D con 1
;--------------------------------------------------------------------
;[SERVO OFFSETS]	deg * 166.6
Base_Offset			con 0
Shoulder_Offset		con 0
Elbow_Offset 		con 0
Wrist_Offset 		con 0
Gripper_Offset 		con 0
WristRotate_Offset 	con 0
;[MOTOR NEUTRAL OFFSET]
RightChNeutral		con 1500
LeftChNeutral		con 1500
;--------------------------------------------------------------------
;[TABLES]
GetACos bytetable 	255,253,252,251,250,248,247,246,244,243,242,241,239,238,237,235,234,233,232,230,229,228,226,|
					225,224,223,221,220,219,217,216,215,213,212,211,209,208,207,206,204,203,201,200,199,197,196,|
					195,193,192,191,189,188,186,185,184,182,181,179,178,177,175,174,172,171,169,168,166,165,163,|
					162,160,159,157,156,154,153,151,149,148,146,145,143,141,140,138,136,135,133,131,129,128,126,|
					124,122,120,118,116,115,113,111,109,106,104,102,100,98,96,93,91,89,86,83,81,78,75,73,70,66,63,|
					60,56,52,48,44,38,33,26,15,0
;--------------------------------------------------------------------
;[CONSTANTS]
c2DEC		con 100
c4DEC		con 10000

TRUE 		con 1
FALSE 		con 0

BUTTON_DOWN con 0
BUTTON_UP 	con 1

ROVER_MODE	con 0
ARM_MODE	con 1

TANK_MODE		con 0
ONE_STICK_MODE	con 1

;calibrate steps per degree.
stepsperdegree1 con 1666 ; steps per degree in 10ths...
;--------------------------------------------------------------------
;[PS2 Controller]
PS2DAT 		con P12		;PS2 Controller DAT (Brown)
PS2CMD 		con P13		;PS2 controller CMD (Orange)
PS2SEL 		con P14		;PS2 Controller SEL (Blue)
PS2CLK 		con P15		;PS2 Controller CLK (White)
PadMode 	con $79
;--------------------------------------------------------------------
;[PIN NUMBERS]
BasePin 		con P0	;Base Connection
ShoulderPin 	con P1	;Shoulder Connection
ElbowPin  		con P2	;Elbow Connection
WristPin		con P3	;Wrist Connection
GripperPin		con P4	;Gripper Connection
WristRotatePin	con P5	;Wrist Rotation Connection (Optional)

RightChannelPin	con P10	;Sabertooth Motor driver Right channel
LeftChannelPin  con P11	;Sabertooth Motor driver Left channel

GripperFSRPin	con P16	;Force Sensing Resistor for the Gripper
;--------------------------------------------------------------------
;[ANALOG SWITCHING POINT]
GripperHoldPressure	con 800 ;+/-1000 = neutral
;--------------------------------------------------------------------
;[MIN/MAX ANGLES] Degrees * 10
Base_MIN		con -900	;Mechanical limits of the Base
Base_MAX		con 900

Shoulder_MIN	con -780	;Mechanical limits of the Shoulder
Shoulder_MAX	con 900

Elbow_MIN		con -710	;Mechanical limits of the Elbow
Elbow_MAX		con 450

Wrist_MIN		con -900	;Mechanical limits of the Wrist
Wrist_MAX		con 900

Gripper_MIN		con -720	;Mechanical limits of the Gripper
Gripper_MAX		con 680

WristRotate_MIN	con -760	;Mechanical limits of the WristRotate
WristRotate_MAX	con 830
;--------------------------------------------------------------------
;[BODY DIMENSIONS]
BaseLength		con 69	;Length of the Base [mm]

#IFDEF AL5A
  HumerusLength		con 94	;Length of the Humerus [mm]
  UlnaLength		con 108	;Length of the Ulna [mm]
  InitWristPosX 	con 20	;Init position X
  InitWristPosY 	con 145 ;Init position Y
  InitGripperAngle1 con -400;Init Angle gripper  
  
#ELSEIFDEF AL5B
  HumerusLength		con 119	;Length of the Humerus [mm]
  UlnaLength		con 127	;Length of the Ulna [mm]
  InitWristPosX 	con 30	;Init position X
  InitWristPosY 	con 130 ;Init position Y
  InitGripperAngle1 con -450;Init Angle gripper 
   
#ELSEIFDEF AL5C
  HumerusLength		con 156	;Length of the Humerus [mm]
  UlnaLength		con 154	;Length of the Ulna [mm]
  InitWristPosX 	con 35	;Init position X
  InitWristPosY 	con 145 ;Init position Y
  InitGripperAngle1 con -450;Init Angle gripper
  
#ELSEIFDEF AL5D
  HumerusLength		con 147	;Length of the Humerus [mm]
  UlnaLength		con 187	;Length of the Ulna [mm] 
  InitWristPosX 	con 55	;Init position X
  InitWristPosY 	con 120 ;Init position Y
  InitGripperAngle1 con -550;Init Angle gripper
#ENDIF
;--------------------------------------------------------------------
;[REMOTE]
DeadZone		con 4	;The deadzone for the analog input from the remote
;====================================================================
;[ANGLES]
BaseAngle1 			var sword	;Actual Angle of the Base servo
ShoulderAngle1 		var sword	;Actual Angle of the Shoulder servo
ElbowAngle1  		var sword	;Actual Angle of the Elbow servo
WristAngle1			var sword	;Actual Angle of the Wrist servo
Gripper1			var sword	;Actual Angle of the Gripper servo
WristRotateAngle1	var sword	;Actual Angle of the Wrist Rotation servo
;--------------------------------------------------------------------
;[POSITIONS]
WristPosX			var slong	;Position of the Gripper used for input
WristPosY			var slong
GlobalGripperAngle1	var sword	;Angle of the Gripper used for input
;--------------------------------------------------------------------
;[Gripper]
GripperPressure		var word	;Analog value from the FSR input
;--------------------------------------------------------------------
;[IR SCANNER]
IRDistance 			var word	;Analog value from the IR sensor
IRSweepDir 			var bit		;Sweep direction 0=L/R, 1=R/L
IRSweepPos 			var float	;Current position from the IR Servo
;--------------------------------------------------------------------
;[ROVER]
SteeringMode		var bit		;Switch between OneStick or Tank Steering
Gear				var byte	;Gear to set the motor speed 1 to 4
LStickX				var sByte	;Stick position to calculate one stick steering
LStickY				var sByte	;Stick position to calculate one stick steering
RStickX				var sByte	;Stick position to calculate one stick steering
RStickY				var sByte	;Stick position to calculate one stick steering
LDrive				var sword	;Left drive channel
RDrive				var sword	;Right drive channel
RightChannel		var sword	;Output for the Right channel
LeftChannel			var sword	;Output for the Left channel
;--------------------------------------------------------------------
;[VARIABLES]

;GetArcCos
Cos4			var sword		;Input Cosinus of the given Angle, decimals = 4
AngleRad4		var sword		;Output Angle in radians, decimals = 4

;GetArcTan2
ArcTanX 		var sword		;Input X
ArcTanY 		var sword		;Input Y
ArcTan4			var slong		;Output ARCTAN2(X/Y)

;Arm Inverse Kinematics
IKWristPosX	    	var slong	;Input position of the Wrist X
IKWristPosY	    	var slong	;Input position of the Wrist Y
IKGripperAngle1		var slong	;Input Angle of the Gripper referring to the base
IKSW2				var long	;Length between Shoulder and Wrist
IKA14		    	var long	;Angle of the line S>W with respect to the ground in radians
IKA24		    	var long	;Angle of the line S>W with respect to the humerus in radians
Temp1				var long
Temp2				var long
IKSolutionWarning 	var bit		;Output true if the solution is NEARLY possible
IKSolutionError		var bit		;Output true if the solution is NOT possible
IKWristAngle1   	var sword	;Output Angle of Wrist in degrees
IKElbowAngle1   	var sword	;Output Angle of Elbow in degrees
IKShoulderAngle1   	var sword	;Output Angle of Shoulder in degrees
IKBaseAngle1		var sword	;Output Angle of Base in degrees
;--------------------------------------------------------------------
;[Ps2 Controller]
DualShock 	var Byte(7)
LastButton 	var Byte(2)
DS2Mode 	var Byte
PS2Index	var byte

;[Speed limitation] ; variables to limit max angle step to limit speed
TargetWristAngle1	var sword
MaxStepWristAngle	con 40
TargetGripper1		var sword
MaxStepGripper		con 40
;--------------------------------------------------------------------
;[GLOBAL]
ControlMode 	var bit			;Switch Between Rover and Arm
RoverOn			var bit			;Switch to turn on the Rover
Prev_RoverOn	var bit			;Previous loop state
ArmOn	 		var bit			;Switch to turn on the Arm
Prev_ArmOn		var bit			;Previous loop state 
;====================================================================
;Interrupt init 
interruptenable var bit	;Memory to see if the interrupts are enabled
ENABLEHSERVO	;Enable Interrupts for HServo

low RightChannelPin	
low LeftChannelPin 
;====================================================================
;[INIT]

;Initialize servo positions
interruptenable=false
Gosub InitArm

;Current gear
Gear = 3

;PS2 controller
high PS2CLK
gosub Ps2Init
LastButton(0) = 255
LastButton(1) = 255

ArmOn = FALSE
RoverOn = FALSE
;====================================================================
;[MAIN]	
main:

#IF EnableFSR=1
	;Read analog input
	adin GripperFSRPin, GripperPressure
#ELSE
	GripperPressure=1000
#ENDIF

  ;Read remote input
  GOSUB Ps2Input
  
;[ROVER CALCULATIONS]
 
  ;Drive Rover
  IF RoverOn THEN ;AND ArmOn=FALSE THEN
    IF Prev_RoverOn=0 THEN
      Sound P9,[60\2500,80\3000,100\3500]
    ENDIF
    
    IF SteeringMode = TANK_MODE THEN	;Tank steering mode
      RDrive = -RStickX
      LDrive = -LStickX     
    ELSE								;One stick steering mode
      IF RStickX>=0 AND RStickY>=0 THEN ;Quadrant 1
        RDrive = -(RStickX - RStickY)
        LDrive = -(RStickX MIN RStickY)
        
      ELSEIF RStickX<0 AND RStickY>=0  ;Quadrant 2
        RDrive = -(RStickX + RStickY)
        LDrive = -(RStickX MAX RStickY)
        
      ELSEIF RStickX<0 AND RStickY<0  ;Quadrant 3
        RDrive = -(RStickX MAX RStickY)
        LDrive = -(RStickX - RStickY)
      
      ELSEIF RStickX>=0 AND RStickY<0  ;Quadrant 4
        RDrive = -(RStickX MIN -RStickY)
        LDrive = -(RStickX + RStickY)      
      
      ELSE
        RDrive = 0
        LDrive = 0      
      ENDIF

    ENDIF
    
    ;Multiply speed with the selected gear
    RDrive = ((RDrive MAX 125) MIN -125) * 2*Gear 
    LDrive = ((LDrive MAX 125) MIN -125) * 2*Gear 
    
	;Send pulse to the Sabertooth
  	pulsout RightChannelPin, (RDrive+RightChNeutral*2)	; *0.5ms
  	pulsout LeftChannelPin,  (LDrive+LeftChNeutral*2)
  	pause 20
  ELSE
  	;Send pulse to stop the Sabertooth
    IF Prev_RoverOn THEN
      Sound P9,[100\5000,80\4500,60\4000]
	  pulsout RightChannelPin, RightChNeutral*2	; *0.5ms
	  pulsout LeftChannelPin,  LeftChNeutral*2
	  pause 20
    ENDIF
  ENDIF
   
  ;Store previous ArmOn State
  IF RoverOn THEN
    Prev_RoverOn = 1
  ELSE
    Prev_RoverOn = 0
  ENDIF
  

;[ARM CALCULATIONS]

  ;Inverse Kinematic calculations
  GOSUB ArmIK [WristPosX, WristPosY, GlobalGripperAngle1]  
  ShoulderAngle1 = -IKShoulderAngle1+900
  ElbowAngle1 = IKElbowAngle1+900
  WristAngle1 = -IKWristAngle1

  GOSUB CheckLimits

  GOSUB WaitForPrevMove ;Wait for servos to finish the previous move
  
  IF ArmOn THEN  
    IF ArmOn AND Prev_ArmOn=0 THEN
      Sound P9,[60\4000,80\4500,100\5000]
	  gosub InitArm
	  
	  GlobalGripperAngle1=0
  	ENDIF
  
	GOSUB Movement[-BaseAngle1, -ShoulderAngle1, -ElbowAngle1, -WristAngle1, -Gripper1, -WristRotateAngle1, 50]
  ELSE
  
    ;Turn the bot off
    IF Prev_ArmOn THEN
    
	  gosub InitArm    
      Sound P9,[100\5000,80\4500,60\4000]
         
    ENDIF
  ENDIF
  
  ;Store previous ArmOn State
  IF ArmOn THEN
    Prev_ArmOn = 1
  ELSE
    Prev_ArmOn = 0
  ENDIF
  
goto main
;====================================================================
;[INITARM] Initialization for the AL5x Arm
InitArm:

  ;Gripper Positions
  WristPosX = InitWristPosX		;Start positions of the Gripper
  WristPosY = InitWristPosY
  GlobalGripperAngle1 = InitGripperAngle1
  
  ;Inverse Kinematic calculations
  GOSUB ArmIK [WristPosX, WristPosY, GlobalGripperAngle1]  
  ShoulderAngle1 = -IKShoulderAngle1+900
  ElbowAngle1 = IKElbowAngle1+900
  WristAngle1 = -IKWristAngle1

  GOSUB CheckLimits
    
  IF interruptenable=FALSE THEN
    InterruptEnable=TRUE
    ENABLE
    ;Movement time needs to be zero when first initializing HSERVO
    gosub movement [0,  -ShoulderAngle1,  -ElbowAngle1,  -WristAngle1,  0,  0,  0]   
    
  ELSE
    gosub movement [0,  -ShoulderAngle1,  -ElbowAngle1,  -WristAngle1,  0,  0,  600]    
  ENDIF
  
  pause 1000

return
;--------------------------------------------------------------------
;[PS2INIT] Initialization for the PS2 remote
Ps2Init:
  low PS2SEL
  shiftout PS2CMD,PS2CLK,FASTLSBPRE,[$1\8]
  shiftin PS2DAT,PS2CLK,FASTLSBPOST,[DS2Mode\8]
  high PS2SEL
  pause 1
  
  if DS2Mode <> PadMode THEN
	low PS2SEL
	shiftout PS2CMD,PS2CLK,FASTLSBPRE,[$1\8,$43\8,$0\8,$1\8,$0\8] ;CONFIG_MODE_ENTER
	high PS2SEL
	pause 1

	low PS2SEL
	shiftout PS2CMD,PS2CLK,FASTLSBPRE,[$01\8,$44\8,$00\8,$01\8,$03\8,$00\8,$00\8,$00\8,$00\8] ;SET_MODE_AND_LOCK
	high PS2SEL
	pause 1

	low PS2SEL
	shiftout PS2CMD,PS2CLK,FASTLSBPRE,[$01\8,$4F\8,$00\8,$FF\8,$FF\8,$03\8,$00\8,$00\8,$00\8] ;SET_DS2_NATIVE_MODE
	high PS2SEL
	pause 1

	low PS2SEL
	shiftout PS2CMD,PS2CLK,FASTLSBPRE,[$01\8,$43\8,$00\8,$00\8,$5A\8,$5A\8,$5A\8,$5A\8,$5A\8] ;CONFIG_MODE_EXIT_DS2_NATIVE
	high PS2SEL
	pause 1

	low PS2SEL
	shiftout PS2CMD,PS2CLK,FASTLSBPRE,[$01\8,$43\8,$00\8,$00\8,$00\8,$00\8,$00\8,$00\8,$00\8] ;CONFIG_MODE_EXIT
	high PS2SEL
	pause 100
		
	sound P9,[100\4000, 100\4500, 100\5000]
	
	goto Ps2Init ;Check if the remote is initialized correctly
  ENDIF
return
;--------------------------------------------------------------------
;[PS2Input] reads the input data from the Wiiremote and processes the
;data to the parameters.
Ps2Input:

  low PS2SEL
  shiftout PS2CMD,PS2CLK,FASTLSBPRE,[$1\8,$42\8]	
  shiftin PS2DAT,PS2CLK,FASTLSBPOST,[DualShock(0)\8, DualShock(1)\8, DualShock(2)\8, DualShock(3)\8, |
  	DualShock(4)\8, DualShock(5)\8, DualShock(6)\8]
  high PS2SEL
  pause 10	
  
  ;Switch between Rover mode and Arm mode
  IF (DualShock(1).bit0 = 0) and LastButton(0).bit0 THEN ;Select Button test  
    IF ControlMode = ROVER_MODE THEN      
      ControlMode = ARM_MODE
      ArmOn = TRUE
	  GlobalGripperAngle1=0 ;Put gripper horizontal
      IF Prev_ArmOn=TRUE THEN ;Only beep when the arm is already on
      	sound P9,[50\4000,  50\5000]
     ENDIF
     
    ELSE   
      ControlMode = ROVER_MODE
      RoverOn=TRUE
      IF Prev_RoverOn=TRUE THEN ;Only beep when the rover is already on 
        sound P9,[50\5000,  50\4000]
      ENDIF
    ENDIF
  ENDIF	
  
  ;Rover Mode
  IF ControlMode = ROVER_MODE THEN
    IF (DualShock(1).bit3 = 0) and LastButton(0).bit3 THEN	;Start Button test
	  IF(RoverOn) THEN
	    ;Turn Rover off
	    RoverOn = False
	  ELSE
	  
	    ;Turn Rover on
	    RoverOn = True	
	  ENDIF
	ENDIF
	
	IF RoverOn THEN
	
	  IF ABS(Dualshock(5) - 128) > DeadZone THEN ;Left Stick L/R
	    LStickY = (Dualshock(5) - 128) ;Wheels
	  ELSE
	    LStickY=0	    
	  ENDIF
	
	  IF ABS(Dualshock(6) - 128) > DeadZone THEN ;Left Stick U/D
	    LStickX = -(Dualshock(6) - 127) ;Wheels
	  ELSE
	    LStickX=0
	  ENDIF
	  
	  IF ABS(Dualshock(3) - 128) > DeadZone THEN ;Right Stick L/R
        RStickY = (Dualshock(3) - 128) ;Wheels
      ELSE
	    RStickY=0
	  ENDIF
    
	  IF ABS(Dualshock(4) - 128) > DeadZone THEN ;Right Stick U/D
	    RStickX = -(Dualshock(4) - 127) ;Wheels
	  ELSE
	    RStickX=0
	  ENDIF	 
	
	  IF (DualShock(1).bit4 = 0) and LastButton(0).bit4 THEN	;D-Up Button test
	    IF Gear < 4 THEN
	      Gear = Gear + 1
	      Sound P9,[50\4000]
	    ELSE
	      Sound P9,[50\4000, 50\5000]
	    ENDIF
	  ENDIF
		
	  IF (DualShock(1).bit6 = 0) and LastButton(0).bit6 THEN	;D-Down Button test
	    IF Gear > 1 THEN
	      Gear = Gear - 1
	      Sound P9,[50\4000]
	    ELSE
	      Sound P9,[50\5000, 50\4000]
	    ENDIF
	  ENDIF
	  
	  IF (DualShock(1).bit2 = 0) and LastButton(0).bit2 THEN	;R3 Button test
	    sound p9, [50\4000]
	    IF SteeringMode=ONE_STICK_MODE THEN
	  	  SteeringMode = TANK_MODE
	    ELSE
	      SteeringMode = ONE_STICK_MODE
	    ENDIF
	  ENDIF	
	  	
	ENDIF
  ENDIF

  ;Arm Mode
  IF ControlMode = ARM_MODE THEN
  
    IF (DualShock(1).bit3 = 0) and LastButton(0).bit3 THEN	;Start Button test
	  IF(ArmOn) THEN
	    ;Turn off
	    ArmOn = False

	    ;Reset Gripper Positions
	    WristPosX=55		;Reset IK postitions of the Gripper
	    WristPosY=100
	  
	    BaseAngle1=0		;Reset servo angles.
	    GlobalGripperAngle1=-550
	    TargetGripper1=0	    	  
	    Gripper1=0
		TargetWristAngle1=0
	    WristRotateAngle1=0
	  ELSE
	  
	    ;Turn on
	    ArmOn = True	
	  
	    ;Put gripper horizontal
	    GlobalGripperAngle1=0
	  ENDIF
    ENDIF	

    IF ArmOn THEN

;	  IF (DualShock(1).bit4 = 0) and LastButton(0).bit4 THEN	;D-Up Button test
;	  ENDIF
		
;	  IF (DualShock(1).bit6 = 0) and LastButton(0).bit6 THEN	;D-Down Button test
;	  ENDIF
	
;	  IF (DualShock(1).bit7 = 0) and LastButton(0).bit7 THEN	;D-Left Button test
;	  ENDIF
	
;	  IF (DualShock(1).bit5 = 0) and LastButton(0).bit5 THEN	;D-Right Button test
;	  ENDIF

;	  IF (DualShock(2).bit5 = 0) and LastButton(1).bit5 THEN	;Circle Button test
;	  ENDIF	
	
;	  IF (DualShock(2).bit7 = 0) and LastButton(1).bit7 THEN	;Square Button test
;	  ENDIF	
	
	  IF (DualShock(2).bit4 = 0) and LastButton(1).bit4 THEN	;Triangle Button test
	    TargetWristAngle1 = 0 ;Put the wrist rotate servo to the middle position
	  ENDIF	
	  
	  IF (DualShock(2).bit6 = 0) and LastButton(1).bit6 THEN	;Cross Button test
	    IF (Gripper1 <> Gripper_MAX) THEN
	      TargetGripper1 = Gripper_MAX ;Fully open Gripper
#IF EnableFSR=1	 	      
	    ELSE
	      TargetGripper1 = Gripper_MIN
#ENDIF	  	      
	    ENDIF
	  ENDIF	
			
	  IF (DualShock(2).bit2 = 0) AND (GripperPressure>GripperHoldPressure) THEN	;L1 Button test
	    TargetGripper1 = (TargetGripper1-40) min Gripper_MIN ;Gripper Close
	  ENDIF
	
	  IF (DualShock(2).bit0 = 0)THEN ;L2 Button test
	    TargetGripper1 = (TargetGripper1+40) max Gripper_MAX ;Gripper Open
	  ENDIF		
	
	  IF (DualShock(2).bit3 = 0) THEN	;R1 Button test
	    TargetWristAngle1 = (TargetWristAngle1-40) min WristRotate_MIN ;Wrist Rotate CW
 	  ENDIF

	  IF (DualShock(2).bit1 = 0) THEN ;R2 Button test
	    TargetWristAngle1 = (TargetWristAngle1+40) max WristRotate_MAX ;Wrist Rotate CCW
	  ENDIF	

	  IF ABS(Dualshock(5) - 128) > DeadZone THEN ;Left Stick L/R
	    IF IKSolutionWarning THEN ;Allow only movement to the base
	      WristPosX = WristPosX -((Dualshock(5) - 128)/18) MIN 0 ;X Position
	    ELSE  
	      WristPosX = WristPosX -(Dualshock(5) - 128)/18 ;X Position
	    ENDIF
	  ENDIF
	   
	  IF ABS(Dualshock(6) - 128) > DeadZone THEN ;Left Stick U/D
	    IF IKSolutionWarning THEN
	      IF WristPosY > BaseLength THEN ;Allow only downward movement
	        WristPosY = WristPosY -((Dualshock(6) - 128)/18) MIN 0	        
	      ELSE ;Allow only upward movement
	        WristPosY = WristPosY -((Dualshock(6) - 128)/18) MAX 0      
	      ENDIF
	    ELSE
	      WristPosY = WristPosY -(Dualshock(6) - 128)/18 ;Y Position
	    ENDIF
	  ENDIF
  
	  IF ABS(Dualshock(3) - 128) > DeadZone THEN ;Right Stick L/R
        BaseAngle1 = BaseAngle1 -(Dualshock(3) - 128)/4 ;Base Rotation
	  ENDIF
	  
	  IF ABS(Dualshock(4) - 128) > DeadZone THEN ;Right Stick U/D
	    GlobalGripperAngle1 = GlobalGripperAngle1-(Dualshock(4) - 128)/2 ;Gripper Angle
	  ENDIF
	  	
	  ;Limit wristrotate max speed  
	  IF (ABS(WristRotateAngle1-TargetWristAngle1) > MaxStepWristAngle) THEN
	    IF ((WristRotateAngle1-TargetWristAngle1) >= 0) THEN
	      WristRotateAngle1 = WristRotateAngle1 - MaxStepWristAngle
	  	ELSE
	      WristRotateAngle1 = WristRotateAngle1 + MaxStepWristAngle	  	
	  	ENDIF
	  ELSE
	  	WristRotateAngle1 = TargetWristAngle1
	  ENDIF
	  
	  ;Limit gripper max speed
	  IF (ABS(Gripper1-TargetGripper1) > MaxStepGripper) THEN
	    IF ((Gripper1-TargetGripper1) >= 0) THEN
	      IF (GripperPressure>GripperHoldPressure) THEN
	        Gripper1 = Gripper1 - MaxStepGripper
	      ENDIF
	  	ELSE
	      Gripper1 = Gripper1 + MaxStepGripper	  	
	  	ENDIF
	  ELSE
	  	Gripper1 = TargetGripper1
	  ENDIF
	  
	  
    ENDIF
  ENDIF
  
  LastButton(0) = DualShock(1)
  LastButton(1) = DualShock(2)
return
;--------------------------------------------------------------------
;[GETARCCOS] Get the sinus and cosinus from the angle +/- multiple circles
;Cos4    	- Input Cosinus
;AngleRad4 	- Output Angle in AngleRad4
GetArcCos[Cos4]

  Cos4 = (Cos4 max c4DEC) min -c4DEC
  IF Cos4>=0 THEN
    AngleRad4 = GetACos(Cos4/78+1) ;78=table resolution (1/127)
    AngleRad4 = AngleRad4*62 ;62=acos resolution (pi/2/255)
  ELSE
    AngleRad4 = GetACos(-Cos4/78+1)  
    AngleRad4 = 31416-AngleRad4*62 ;62=acos resolution (pi/2/255)
  ENDIF

return AngleRad4
;--------------------------------------------------------------------
;[ARCTAN2] Gets the Inverse Tangus from X/Y with the where Y can be zero or negative
;ArcTanX 		- Input X
;ArcTanY 		- Input Y
;ArcTan4  		- Output ARCTAN2(X/Y)
GetArcTan2 [ArcTanX, ArcTanY]
	IF(ArcTanX = 0) THEN	; X=0 -> 0 or PI
		IF(ArcTanY >= 0) THEN
			ArcTan4 = 0
		ELSE
			ArcTan4 = 31415
		ENDIF
	ELSE	
		IF(ArcTanY = 0) THEN	; Y=0 -> +/- Pi/2
			IF(ArcTanX > 0) THEN
				ArcTan4 = 15707
			ELSE
				ArcTan4 = -15707
			ENDIF
		ELSE
			IF(ArcTanY > 0) THEN	;ARCTAN(X/Y)
				ArcTan4 = TOINT(FATAN(TOFLOAT(ArcTanX) / TOFLOAT(ArcTanY))*10000.0)
			ELSE	
				IF(ArcTanX > 0) THEN	;ARCTAN(X/Y) + PI	
					ArcTan4 = TOINT(FATAN(TOFLOAT(ArcTanX) / TOFLOAT(ArcTanY))*10000.0) + 31415
				ELSE					;ARCTAN(X/Y) - PI	
					ArcTan4 = TOINT(FATAN(TOFLOAT(ArcTanX) / TOFLOAT(ArcTanY))*10000.0) - 31415
				ENDIF
			ENDIF
		ENDIF
	ENDIF
return ArcTan4
;-------------------------------------------------------------------- 
;[ARM INVERSE KINEMATICS] Calculates the angles of the Humerus and Ulna for the given position of the Gripper
;IKGripperPosX		- Input position of the Gripper X
;IKGripperPosY		- Input position of the Gripper Y
;IKGripperPosZ		- Input Position of the Gripper Z
;IKGripperAngle1	- Input Angle of the Gripper referring to the base
;IKSolution			- Output true IF the solution is possible
;IKSolutionWarning 	- Output true IF the solution is NEARLY possible
;IKSolutionError	- Output true IF the solution is NOT possible
;IKUlnaAngle1  		- Output Angle of Ulna in degrees
;IKHumerusAngle1   	- Output Angle of Humerus in degrees
;IKBaseAngle1		- Output Angle of Base in degrees
ArmIK [IKWristPosX, IKWristPosY, IKGripperAngle1] 
    
  ;IKSW - Length between Shoulder and wrist 
  IKSW2 = SQR((((IKWristPosY-BaseLength)*(IKWristPosY-BaseLength))+(IKWristPosX*IKWristPosX))*(c4DEC/4))   ; KJE - value 1/2 what it should be now    
    
  ;IKA1 - Angle between SW line and the ground in rad 
  GOSUB GetArcTan2 [(IKWristPosY-BaseLength), IKWristPosX], IKA14 
    
  ;IKA2 - Angle of the line S>W with respect to the humerus in radians 
  Temp1 = (((HumerusLength*HumerusLength) - (UlnaLength*UlnaLength))*(c4DEC/4) + (IKSW2*IKSW2))         ; KJE - Value is 1/4 of what it should be. 
  Temp2 = ((2*HumerusLength)*c2DEC * IKSW2)                                                ; KJE - value is 1/2 
  GOSUB GetArcCos [(Temp1 / (Temp2/c4DEC)) *2 ], IKA24                                       ; KJE - corrected for 1/4value / 1/2 value 

  ;Elbow Angle 
  Temp1 = (((HumerusLength*HumerusLength) + (UlnaLength*UlnaLength))*(c4DEC/4) - (IKSW2*IKSW2))         ; KJE 1/4 
  Temp2 = (2*HumerusLength*Ulnalength) 
  GOSUB GetArcCos [(Temp1 / Temp2) * 4]                                                   ; kje fixed for 1/4 
  IKElbowAngle1 = -(1800-AngleRad4*180/3141) 

  ;Shoulder Angle 
  IKShoulderAngle1 = (IKA14 + IKA24) * 180 / 3141 

  ;Wrist Angle 
  IKWristAngle1 = IKGripperAngle1-IKElbowAngle1-IKShoulderAngle1 
       
  ;Set the Solution quality
  IKSolutionWarning = ((IKSW2*2) > ((UlnaLength+HumerusLength-40)*c2DEC)) ; kje fixed for 1/2 
  IKSolutionError = ((IKSW2*2) > ((UlnaLength+HumerusLength)*c2DEC)) ; kje fixed for 1/2
    
return
;--------------------------------------------------------------------
;[CHECK ANGLES] Checks the mechanical limits of the servos
CheckLimits:
  BaseAngle1  = 	(BaseAngle1  min Base_MIN) max Base_MAX
  ShoulderAngle1 = 	(ShoulderAngle1 min Shoulder_MIN) max Shoulder_MAX
  ElbowAngle1 = 	(ElbowAngle1 min Elbow_MIN)  max Elbow_MAX
  WristAngle1  = 	(WristAngle1  min Wrist_MIN) max Wrist_MAX
  Gripper1 = 		(Gripper1 min Gripper_MIN) max Gripper_MAX
  WristRotateAngle1 =(WristRotateAngle1 min WristRotate_MIN)  max WristRotate_MAX
return
;--------------------------------------------------------------------
;[MOVEMENT] Does a servo groupmove to ensure that all servos reach
;the destination at the same time. 
BasePosition1			var slong
ShoulderPosition1 		var slong
ElbowPosition1 			var slong
WristPosition1			var slong
GripperPosition1		var slong
WristRotatePosition1	var slong
MaxSpeed 				var slong 
_mindex 				var byte
_mPrev_BasePos1			var slong
_mPrev_ShoulderPos1 	var slong
_mPrev_ElbowPos1  		var slong
_mPrev_WristPos1		var slong
_mPrev_GripperPos1		var slong
_mPrev_WristRotatePos1	var slong
_mBaseSpeed				var slong
_mShoulderSpeed 		var slong
_mElbowSpeed  			var slong
_mWristSpeed			var slong
_mGripperSpeed			var slong
_mWristRotateSpeed		var slong
_mMaxSpeed1				var slong
_mlongestmove1 			var slong 
Movement [BasePosition1,ShoulderPosition1,ElbowPosition1,WristPosition1,GripperPosition1,WristRotatePosition1,MaxSpeed] 

  if(MaxSpeed<>0)then 
	;Get the longest move
    gosub getlongest[BasePosition1	 -_mPrev_BasePos1, | 
					 ShoulderPosition1 -_mPrev_ShoulderPos1, | 
					 ElbowPosition1	 -_mPrev_ElbowPos1, | 
					 WristPosition1	 -_mPrev_WristPos1, | 
					 GripperPosition1	 -_mPrev_GripperPos1, | 
					 WristRotatePosition1 -_mPrev_WristRotatePos1], _mlongestmove1 

  ; kje - converted to return maxspeed *10 - 
  _mMaxspeed1 = ((_mlongestmove1*stepsperdegree1)/(MaxSpeed/2)) 

  ;Calculate speed for each servo
  gosub getspeed[BasePosition1,		_mPrev_BasePos1,	_mlongestmove1,		_mMaxSpeed1],	_mBaseSpeed
  gosub getspeed[ShoulderPosition1,	_mPrev_ShoulderPos1,_mlongestmove1,		_mMaxSpeed1],	_mShoulderSpeed 
  gosub getspeed[ElbowPosition1,	_mPrev_ElbowPos1,	_mlongestmove1,		_mMaxSpeed1],	_mElbowSpeed 
  gosub getspeed[WristPosition1,	_mPrev_WristPos1,	_mlongestmove1,		_mMaxSpeed1],	_mWristSpeed
  gosub getspeed[GripperPosition1,	_mPrev_GripperPos1,	_mlongestmove1,		_mMaxSpeed1],	_mGripperSpeed
  gosub getspeed[WristRotatePosition1,	_mPrev_WristRotatePos1,_mlongestmove1,_mMaxSpeed1],	_mWristRotateSpeed      

  else 
	_mBaseSpeed = 0
	_mShoulderSpeed = 0
	_mElbowSpeed = 0
	_mWristSpeed = 0
	_mGripperSpeed = 0
	_mWristRotateSpeed = 0
  endif 
   
   ;Drive servos
   hservo [BasePin\		(BasePosition1*stepsperdegree1)/100 + Base_Offset\_mBaseSpeed, | 
           ShoulderPin\	(ShoulderPosition1*stepsperdegree1)/100 + Shoulder_Offset\_mShoulderSpeed, | 
           ElbowPin\	(ElbowPosition1*stepsperdegree1)/100 + Elbow_Offset\_mElbowSpeed, | 
           WristPin\	(WristPosition1*stepsperdegree1)/100 + Wrist_Offset\_mWristSpeed, | 
           GripperPin\	(GripperPosition1*stepsperdegree1)/100 + Gripper_Offset\_mGripperSpeed, | 
           WristRotatepin\(WristRotatePosition1*stepsperdegree1)/100 + WristRotate_Offset\_mWristRotateSpeed] 

return
;--------------------------------------------------------------------
;[WAITFORPREVMOVE] Loop that wait for the previous servo move to be finished
idle 		var byte
finished 	var byte
junk 		var word
WaitForPrevMove
	; Simple loop to check the status of all of servos.  This loop will
	; terminate when all of the servos say that they are idle.  I.E.
	; they reached their new destination.   
	do 
		finished = hservoidle(BasePin) and hservoidle(ShoulderPin) and hservoidle(ElbowPin) and hservoidle(WristPin) 	and hservoidle (GripperPin) and hservoidle(WristRotatePin)
		
	while (NOT finished)	

	if(NOT finished)then WaitForPrevMove

	_mPrev_BasePos1			= BasePosition1
	_mPrev_ShoulderPos1 	= ShoulderPosition1
	_mPrev_ElbowPos1  		= ElbowPosition1
	_mPrev_WristPos1		= WristPosition1
	_mPrev_GripperPos1		= GripperPosition1
	_mPrev_WristRotatePos1	= WristRotatePosition1
	
return
;--------------------------------------------------------------------
;[GETLONGEST] Returns the longest value
one 	var slong 
two 	var slong 
three 	var slong 
four 	var slong 
five 	var slong 
six 	var slong 
GetLongest[one,two,three,four,five,six] 
   if(one<0)then 
      one=-1*one 
   endif 
   if(two<0)then 
      two=-1*two 
   endif 
   if(three<0)then 
      three=-1*three 
   endif 
   if(four<0)then 
      four=-1*four 
   endif 
   if(five<0)then 
      five=-1*five 
   endif 
   if(six<0)then 
      six=-1*six 
   endif 
   if(one<two)then 
      one=two 
   endif 
   if(one<three)then 
      one=three 
   endif 
   if(one<four)then 
      one=four 
   endif 
   if(one<five)then 
      one=five 
   endif 
   if(one<six)then 
      one=six 
   endif 
return one  
;--------------------------------------------------------------------    
;[GetSpeed] Calculate the speed for each servo
newpos1 	var slong 
oldpos1 	var slong 
longest1 	var slong 
maxval1 	var slong 
GetSpeed[newpos1,oldpos1,longest1,maxval1] 

   if(newpos1>oldpos1)then 
      return (((newpos1-oldpos1)*maxval1)/longest1)/10
   endif 

return (((oldpos1-newpos1)*maxval1)/longest1)/10
;--------------------------------------------------------------------
