'Program name: PS2ROV4.BAS
'Author: Jim Frye / Nathan Scherdin (PS2 stuff)
'Modified by Laurent Gay : speed limit control with L1/L2
'Modified by Brian Nave : Normalized analog joysticks, named button variables, positive boolean logic, audio feedback on speed limit
'Modified by Laurent Gay : New PS2 Controller code (more compatible)
'Modified by James Frye : Configured for Atom Pro

' This program allows the remote control of a differentially steered robot 
' from the Sony Play Station 2 game controller. 

' The Scorpion has two modes of operation. The default is the mixer mode. This
' is where you have a Throttle for forward and reverse speed control, and a 
' Steering control for turning. In this mode the left input is throttle and 
' the right input is steering. The other mode is a differential style control. 
' This is where the left and right channels are controled indepentantly to 
' steer like a tank. This program uses the mixing mode. It also requires the 
' BEC (battery eliminator circuit to be dissabled. See the Scorpion manual.

'Connections
'Pin 0	Left Scorpion channel.
'Pin 1	Right Scorpion channel.
'Pin 2	Pan servo.
'Pin 3	Tilt servo.
'Pin 8	NA
'Pin 9	Speaker
'Pin 10	1/0 Controlled from Triangel Button
'Pin 11	1/0 Controlled from "X" button
'---------------
'BotBoard I :
'Pin 4	PS2 Data
'Pin 5	PS2 Command
'Pin 6	PS2 Select
'Pin 7	PS2 Clock
'Pin 12	Gripper up/down servo.
'Pin 13	Gripper rotate servo.
'Pin 14	Gripper open/close servo.
'Pin 15	NA
'---------------
'BotBoard II :
'Pin 7  NA
'Pin 12	PS2 Data
'Pin 13	PS2 Command
'Pin 14	PS2 Select
'Pin 15	PS2 Clock
'Pin 4	Gripper up/down servo.
'Pin 5	Gripper rotate servo.
'Pin 6	Gripper open/close servo.
'---------------

'PlayStation game controller connections.

'PS2 Controller / BotBoard I
'DAT con P4
'CMD con P5
'SEL con P6
'CLK con P7
'GRIP_UD  con p12
'GRIP_Rot con p13
'GRIP_OC  con p14
'PS2 Controller / BotBoard II
DAT con P12
CMD con P13
SEL con P14
CLK con P15
GRIP_UD  con p4
GRIP_Rot con p5
GRIP_OC  con p6

temp	var	byte	' Variable definitions.
buttons	var	word
Lastbuttons var word

bpress1	var	bit
bpress2	var	bit

rhori	var	byte
rvert	var	byte
lhori	var	byte
lvert	var	byte

rhori_null	var	byte
rvert_null	var	byte
lhori_null	var	byte
lvert_null	var	byte

NormalizeValue	var	byte
NormalizeNull	var	byte
DeadBand		con	2

ldrive					var word
rdrive					var word
PanPosition				var	word
TiltPosition			var	word

GripperVertPosition		var	word
GripperTurnPosition		var	word
GripperGripPosition		var	word
servo5	var	word

GEAR var byte

Button_START 	var bit 	
Button_SELECT 	var bit 	

Button_L1 		var bit 	
Button_L2 		var bit 	
Button_R1 		var bit 	
Button_R2 		var bit 	

Button_A 		var bit 	
Button_O 		var bit 	
Button_X 		var bit 	
Button_S 		var bit 	

Dpad_UP 		var bit 	
Dpad_RIGHT 		var bit 	
Dpad_LEFT 		var bit 	
Dpad_DOWN 		var bit 	

GEAR = 3			' Start at Gear 3 (from 1 to 4)
PanPosition = 1500		' Start the servos at mid position.
TiltPosition = 1500
GripperVertPosition 	= 1500
GripperTurnPosition 	= 1500
GripperGripPosition 	= 1500

low	p0				' Ensure pulsout commands are positive going.
low	p1
low	p2
low	p3
low	GRIP_UD
low	GRIP_Rot
low	GRIP_OC

low P10				' Ensure device connected to pin 10 and 11 is off on startup
low P11

high CLK			' Ensure CLK is negative going.

sound 9, [100\880, 100\988, 100\1046, 100\1175]		'four quick ascending notes.

pause 1000

setup		' This section sets the PSX Controller to Analog Mode. (red LED should light)
			' This will work with Sony and Madcatz wired, but not with Medcatz wireless.
			' You must press the analog button on the controller to set to analog mode.
	low SEL
	shiftout CMD,CLK,FASTLSBPRE,[$1\8]
	shiftin DAT,CLK,FASTLSBPOST,[temp\8]
	high SEL
	pause 1	

	if (temp <> $73) and (temp <> $79) then
		low SEL
		shiftout CMD,CLK,FASTLSBPRE,[$1\8,$43\8,$0\8,$1\8,$0\8] ;CONFIG_MODE_ENTER
		high SEL
		pause 1
		
		low SEL
		shiftout CMD,CLK,FASTLSBPRE,[$01\8,$44\8,$00\8,$01\8,$03\8,$00\8,$00\8,$00\8,$00\8] ;SET_MODE_AND_LOCK
		high SEL
		pause 100
		
		low SEL
		shiftout CMD,CLK,FASTLSBPRE,[$01\8,$4F\8,$00\8,$FF\8,$FF\8,$03\8,$00\8,$00\8,$00\8] ;SET_DS2_NATIVE_MODE
		high SEL
		pause 1
		
		;low SEL
		;shiftout CMD,CLK,FASTLSBPRE,[$01\8,$4D\8,$00\8,$00\8,$01\8,$FF\8,$FF\8,$FF\8,$FF\8] ;VIBRATION_ENABLE
		;high SEL
		;pause 1
	
		low SEL
		shiftout CMD,CLK,FASTLSBPRE,[$01\8,$43\8,$00\8,$00\8,$5A\8,$5A\8,$5A\8,$5A\8,$5A\8] ;CONFIG_MODE_EXIT_DS2_NATIVE
		high SEL
		pause 1
		
		low SEL
		shiftout CMD,CLK,FASTLSBPRE,[$01\8,$43\8,$00\8,$00\8,$00\8,$00\8,$00\8,$00\8,$00\8] ;CONFIG_MODE_EXIT
		high SEL
		pause 1
	
		goto setup
	endif

	gosub get_PSX_data
	gosub NullJoysticks
	
main
	gosub get_PSX_data
	
	if Button_START then : gosub NullJoysticks : endif
		
'SET LINEARIZED JOYSTICK DISPLACEMENT BASED ON JOYSTICK CENTER POSITION
	NormalizeValue = lhori : NormalizeNull = lhori_null : Gosub Normalize : lhori = NormalizeValue
	NormalizeValue = lvert : NormalizeNull = lvert_null : Gosub Normalize : lvert = NormalizeValue
	NormalizeValue = rhori : NormalizeNull = rhori_null : Gosub Normalize : rhori = NormalizeValue
	NormalizeValue = rvert : NormalizeNull = rvert_null : Gosub Normalize : rvert = NormalizeValue
		
	'   remove the rems (') to see the NORMALIZED joystick values in terminal 1.
	'	serout S_OUT,i57600,[dec3 lhori\3," ",dec3 lvert\3," ",dec3 rhori\3," ",dec3 rvert\3," ", 13]
	
'SET MAX MOVEMENT SPEED TO ONE OF FOUR CHOICES AND GIVE AUDIO FEEDBACK
	if Button_L1 and (Lastbuttons.bit10 = 0) then 
		GEAR = (GEAR + 1) MAX 4
		for temp = 1 to GEAR: sound 9, [100\880]:pause 10:next		
	elseif BUTTON_L2 and (Lastbuttons.bit8 = 0)
		GEAR = (GEAR - 1) MIN 1
		for temp = 1 to GEAR: sound 9, [100\880]:pause 10:next		
	endif

'	lvert=255-lvert				' Changes vertical up from 0 to 255 and vertical down from 255 to 0.
	lhori=255-lhori

'CALCULATE DRIVE SPEEDS FROM JOYSTICK POSITIONS AND SPEED LIMIT	
	ldrive=((lvert*GEAR) + 1500 - (GEAR *128)) ' Forward / backward
	rdrive=((lhori*3) + 1500 - (3 *128)) ' steering
	
	'   remove the rems (') to see the calculated drive motor speeds in terminal 1.
	'	serout S_OUT,i57600,[dec5 ldrive\5," ",dec5 rdrive\5," ",13]
	
	
	PanPosition  = (PanPosition  + (rhori-127)/5) MAX 2250 MIN 750
	TiltPosition = (TiltPosition + (rvert-127)/5) MAX 2250 MIN 750

	GripperVertPosition = (GripperVertPosition - Dpad_UP*25    + Dpad_DOWN*25) 	MAX 2250 MIN 750 	'Move Gripper Vertically.
	GripperTurnPosition = (GripperTurnPosition - Dpad_RIGHT*25 + Dpad_LEFT*25) 	MAX 2250 MIN 750 	'Rotate Gripper.
	GripperGripPosition = (GripperGripPosition - Button_R2*25  + Button_R1*25) 	MAX 2250 MIN 750 	'Grip Gripper.
	
	'   remove the rems (') to see the gripper and pan positions in terminal 1.
	'	serout S_OUT,i57600,[dec4 GripperVertPosition\4," ",dec4 GripperTurnPosition\4," ",dec4 GripperGripPosition\4," ",dec4 PanPosition\4," ",dec4 TiltPosition\4, 13]
	
' I/O on/off	

	if Button_A and (bpress1=0) then	' This makes a latching output pin that responds to button presses.
		bpress1=1
		toggle p10
	endif
	if (Button_A=0) and bpress1 then
		bpress1=0
	endif 

	if Button_X and (bpress2=0) then	' This makes a latching output pin that responds to button presses.
		bpress2=1
		toggle p11
	endif
	if (Button_X=0) and bpress2 then
		bpress2=0
	endif 


' Send out the servo pulses	
	pulsout 0,(ldrive*2)					' Left Scorpion channel.
	pulsout 1,(rdrive*2)					' Right Scorpion channel.
	pulsout 2,(PanPosition*2)				' Pan servo.
	pulsout 3,(TiltPosition*2)				' Tilt servo.
	pulsout GRIP_UD,(GripperVertPosition*2)		' Gripper up/down servo.
	pulsout GRIP_Rot,(GripperTurnPosition*2)	' Gripper rotate servo.
	pulsout GRIP_OC,(GripperGripPosition*2)		' Gripper open/close servo.
'	pulsout 15,servo5
	pause 20
goto main


get_PSX_data	' This section gets the data from the PSX controller.
				' The first byte is the mode (Temp)
				' The 2 next bytes are the pushbutton data (button1, button2).
				' The 4 next bytes are the analog joystick data (rhori,rvert,lhori,lvert).
	Lastbuttons = buttons
	low SEL
	' Initiate request for data from PSX controller.
	shiftout CMD,CLK,FASTLSBPRE,[$1\8,$42\8]		
	' Temp is the Mode value, it will be dumped. Then it puts the button data into two byte variables.				
	shiftin DAT,CLK,FASTLSBPOST,[temp\8,buttons.LOWBYTE\8,buttons.HIGHBYTE\8,rhori\8,rvert\8,lhori\8,lvert\8]								
	high SEL
	pause 1
	
	buttons = buttons ^ $FFFF	' SO WE CAN USE POSITIVE BOOLEAN LOGIC WHEN EVALUATING BUTTONS
	
	Button_SELECT 	= Buttons.bit0
	Button_START 	= Buttons.bit3
	
	Dpad_UP 		= Buttons.bit4
	Dpad_RIGHT 		= Buttons.bit5
	Dpad_DOWN 		= buttons.bit6
	Dpad_LEFT 		= Buttons.bit7
	
	Button_L2 		= Buttons.bit8
	Button_R2 		= Buttons.bit9
	Button_L1 		= Buttons.bit10
	Button_R1 		= Buttons.bit11
	
	Button_A 		= Buttons.bit12
	Button_O 		= Buttons.bit13
	Button_X 		= Buttons.bit14
	Button_S 		= Buttons.bit15	
	
'   remove the rems (') to see the RAW joystick and Button values in terminal 1.
'	serout S_OUT,i57600,[bin buttons\16," "]
'	serout S_OUT,i57600,[dec3 rhori\3," ",dec3 rvert\3," ",dec3 lhori\3," ",dec3 lvert\3," "]
'	serout S_OUT,i57600,[13]
return

Normalize
	if NormalizeValue < (NormalizeNull-DeadBand) then
		NormalizeValue = (127*NormalizeValue)/(NormalizeNull-DeadBand) MAX 127
	elseif NormalizeValue > (NormalizeNull+DeadBand)
		NormalizeValue = (127*(NormalizeValue-NormalizeNull)/(255-DeadBand-NormalizeNull) + 127) MAX 255
	else
		NormalizeValue = 127
	endif
return	

NullJoysticks  'READ ANALOG JOYSTICKS TO NULL OFF-CENTER VALUES
	rhori_null	= rhori
	rvert_null	= rvert
	lhori_null	= lhori
	lvert_null	= lvert
return
