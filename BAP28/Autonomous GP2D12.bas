'Program name: 4WD1AUTO.BAS
'Author: James Frye

'Connections
'Pin 16	Jumper to battery (VS)
'Pin 17	Left GP2D12 Sensor	(Right facing sensor)
'Pin 18	Right GP2D12 Sensor	(Left facing sensor)
'Pin 19	Rear GP2D12 Sensor
'Pin 0	Left Sabertooth channel.
'Pin 1	Right Sabertooth channel.
'Pin 12	A Button.
'Pin 13	B Button.
'Pin 14	C Button.
'Pin 9	Speaker.

temp		var byte
filter		var word(10)
ir_right	var word
ir_left		var word
ir_rear		var word

LSpeed		var word
RSpeed		var word

minspeed	con 1750
maxspeed	con 1250

LSpeed = 1500
RSpeed = 1500

low p0
low p1

sound 9, [100\880, 100\988, 100\1046, 100\1175]

main
gosub sensor_check

; Numbers lower than 1500 result in forward direction.
; Numbers higher than 1500 result in reverse direction.

LSpeed = (LSpeed - 10) min maxspeed	;accelerates the motors
RSpeed = (RSpeed - 10) min maxspeed

LSpeed = (LSpeed + ir_left) max minspeed	;when something is detected, this decelerates the opposite side
RSpeed = (RSpeed + ir_right) max minspeed

if (ir_rear > 15) then
LSpeed = (LSpeed - ir_rear) min maxspeed	;if something is detected behind the robot, accelerates both sides
RSpeed = (RSpeed - ir_rear) min maxspeed
endif

; Send out the servo pulses	
	pulsout 0,(LSpeed*2)	; Left Sabertooth channel.
	pulsout 1,(RSpeed*2)	; Right Sabertooth channel.
	pause 20

goto main


sensor_check

for temp = 0 to 9
  adin 17, filter(temp)
next
ir_right = 0
for temp = 0 to 9
  ir_right = ir_right + filter(temp)
next
ir_right = ir_right / 85

for temp = 0 to 9
  adin 18, filter(temp)
next
ir_left = 0
for temp = 0 to 9
  ir_left = ir_left + filter(temp)
next
ir_left = ir_left / 85

for temp = 0 to 9
  adin 19, filter(temp)
next
ir_rear = 0
for temp = 0 to 9
  ir_rear = ir_rear + filter(temp)
next
ir_rear = ir_rear / 85


 serout s_out,i38400,["ir_right - ", dec ir_right, " ir_left - ", dec ir_left, " ir_rear - ", dec ir_rear, "LSpeed - ", dec LSpeed, " RSpeed - ", dec RSpeed, 13]

return