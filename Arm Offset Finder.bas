;------------------------------------
; Servo Offset Finder				
;		Written by James Frye		
;									
; Use this program to find 		
; the servo offsets for your		
; Lynxmotion Arm using an Atom Pro	
; and a Bot Board!					
;									
;------------------------------------
;			  How to use			
;									
; Press A to decrease the servo 	
;  offset by 5us					
;									
; Press C to increase the servo	
;  offset by 5us					
;									
; Press B to change which servo is 
;  being manipulated, and to send	
;  data back to the terminal		
;
; The servos are changed in this order:	
;									
;     Base -->  Shoulder -->  Elbow 	
;		^						|	
;		|						|	
;		|						v	
;	 Send <- Wrist <- Gripper <-Wrist
;	  Data	  Rotate	 				
;------------------------------------



enablehservo


;System variables
BasePin			con P0	;Base Connection
ShoulderPin		con P1	;Shoulder Connection
ElbowPin		con P2	;Elbow Connection
WristPin		con P3	;Wrist Connection
GripperPin		con P4	;Gripper Connection
WristRotatePin	con P5	;Wrist Rotation Connection (Optional)
IRScanPin		con P6	;IR Scanner pin (optional)

RMotorPin		con P10	;Sabertooth CH1
LMotorPin		con P11	;Sabertooth CH2

NUMSERVOS   con 9
ServoTable	bytetable BasePin, ShoulderPin, ElbowPin, WristPin, GripperPin, WristRotatePin, IRScanPin, RMotorPin, LMotorPin
swServoVal	var sword
i			var byte

aServoVals	var sword(NUMSERVOS)
Base		var aServoVals(0)
Shoulder	var aServoVals(1)
Elbow		var aServoVals(2)
Wrist		var aServoVals(3)
Gripper		var aServoVals(4)
Rotate		var aServoVals(5)
IRScan		var aServoVals(6)
RMotor		var aServoVals(7)
LMotor		var aServoVals(8)

Base	 = 0
Shoulder = 0
Elbow	 = 0
Wrist	 = 0
Gripper	 = 0
Rotate	 = 0
IRScan	 = 0
RMotor	 = 0
LMotor	 = 0

 currentServo var byte   
 currentServo = 0

;button init
buttonA var bit
buttonB var bit
buttonC var bit

prevA var bit
prevB var bit
prevC var bit

input p12
input p13
input p14


 sound 9,[50\3800, 50\4200, 40\4100]
 
   gosub MoveServos
   
   pause 500

   gosub ShowWhichServo
   gosub MoveServos
   
   
main

 prevA = buttonA
 prevB = buttonB
 prevC = buttonC

 buttonA = in12
 buttonB = in13
 buttonC = in14

 if (buttonA = 0) AND (prevA = 1) then
   sound 9,[50\3800]
 
  if aServoVals(CurrentServo) > -750 then
    aServoVals(CurrentServo) = aServoVals(CurrentServo) - 25
    gosub MoveServos
      else
        sound 9,[150\3500]
  endif
 
  elseif (buttonB = 0) AND (prevB = 1)
    sound 9,[50\(3600 + (currentServo * 100))]
   
    currentServo = currentServo + 1
      if(currentServo = NUMSERVOS) then
        sound 9,[75\3200, 50\3300]
        goto output_data
      endif
    
    gosub ShowWhichServo
    gosub MoveServos
 
  elseif (buttonC = 0) AND (prevC = 1)
    sound 9,[50\4400]
 
    if aServoVals(CurrentServo) < 750 then
      aServoVals(CurrentServo) = aServoVals(CurrentServo) + 25
      gosub MoveServos
      else
        sound 9,[150\3500]
    endif

 endif
 
 goto main



MoveServos:
   hservo [BasePin\Base, ShoulderPin\Shoulder, Elbowpin\Elbow, WristPin\Wrist, Gripperpin\gripper, WristRotatePin\rotate, IRScanPin\IRScan, RmotorPin\RMotor, LMotorPin\LMotor]
return

ShowWhichServo:
   hservo [ServoTable(CurrentServo)\-2500\200]
   hservowait[ServoTable(CurrentServo)]
   hservo [ServoTable(CurrentServo)\2500\200]
   hservowait[ServoTable(CurrentServo)]
   hservo [ServoTable(CurrentServo)\0\128]
   hservowait[ServoTable(CurrentServo)]

return


output_data

 serout s_out,i9600,[";[SERVO OFFSETS]", 13,|
 "Base_Offset			con ", sdec Base, 13,|
 "Shoulder_Offset		con ", sdec Shoulder, 13,|
 "Elbow_Offset			con ", sdec Elbow, 13,|
 "Wrist_Offset			con ", sdec Wrist, 13,|
 "Gripper_Offset		con ", sdec Gripper, 13,|
 "WristRotate_Offset	con ", sdec Rotate, 13,|
 "IRScan_Offset		con ", sdec IRScan, 13,|
 ";[MOTOR NEUTRAL OFFSET]",13,|
 "RightChNeutral		con ", dec (1500 + RMotor), 13,|
 "LeftChNeutral		con ", dec (1500 + LMotor), 13]
 
waitloop

 gosub MoveServos[Base,Shoulder,Wrist,Elbow,Gripper,Rotate,RMotor,LMotor]
 
 buttonB = in13

 if (buttonB = 0) AND (prevB = 1) then
   currentServo = 0 
   sound 9,[200\3800]
   goto main
 endif
 
 prevB = buttonB
 
goto waitloop 