'Program name: 4WD1TST1.BAS
'Author: Jim Frye

' This program tests the motion of the robot using the A, B, and C buttons on
' the Bot Board. Pressing the A button increments steering variable by 10% 
' each press. Pressing the C button decrements steering variable by 10% each 
' press. The B button increments to full forward, then decrements to full 
' reverse speed by 10% each press.

' The Scorpion has two modes of operation. The default is the mixer mode. This
' is where you have a Throttle for forward and reverse speed control, and a 
' Steering control for turning. In this mode the left input is throttle and 
' the right input is steering. The other mode is a differential style control. 
' This is where the left and right channels are controled indepentantly to 
' steer like a tank. 

'Bot Board Jumpers
' Speaker enable
' VS to VL
' A, B, C, button enable
' AX 0-3 power bus to VL

'Connections
' Pin 0  Left Scorpion channel. (Throttle)
' Pin 1  Right Scorpion channel. (Steering)
' Pin 4  A Button.
' Pin 5  B Button.
' Pin 6  C Button.
' Pin 9  Speaker.

temp		var	byte	' Variable definetions.
throttle	var	byte
steering	var	byte
direction	var	bit

throttle = 150		' Do not move.
steering = 150		' Do not turn.
direction = 1

low	p0				' Ensure pulsout commands are positive going.
low	p1

sound 9, [100\880, 100\988, 100\1046, 100\1175]

main:
if in12 = 0 then right_turn		'A button increments steering variable by 10% each press.
if in13 = 0 then throttle_up		'B button increments then decrements speed variable by 10% each press. 
if in14 = 0 then left_turn		'C button decrements steering variable by 10% each press.
' Send out the servo pulses	
	pulsout 0,(throttle*20)	' Left Scorpion channel.
	pulsout 1,(steering*20)	' Right Scorpion channel.
	pause 20
'	serout S_OUT,i57600,["T ", dec throttle, "   S ", dec steering, "   D ", dec direction, 13] ' Remove the rem at the beginning to see in term1.
goto main

throttle_up:
sound 9, [100\880]
if in13 = 0 then throttle_up
  if direction = 0 then throttle_down
    throttle = (throttle -5) min 100
  if throttle = 100 then t_up
goto main 

t_up: 
  direction = 0
goto main

throttle_down:
  if in13 = 0 then throttle_down
    throttle = (throttle +5) max 200
  if throttle = 200 then t_down
goto main 

t_down: 
  direction = 1
goto main

right_turn:
sound 9, [100\988]
 if in12 = 0 then right_turn
  steering = (steering -5) min 100
goto main

left_turn:
sound 9, [100\1046]
 if in14 = 0 then left_turn
  steering = (steering +5) max 200
goto main