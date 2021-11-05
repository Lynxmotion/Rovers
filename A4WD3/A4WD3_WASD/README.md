A4WD3 Rover WASD keyboard example.

In order to use this example you need a serial terminal which doesn't require you to press "Enter" between each keyboard input.
We greatly suggest Putty (https://www.putty.org/)

Controls:<br/>
  This example will take any WASD inputs and will STAY in this mode until you press another key.
  It will not stop by itself so be aware before starting the Rover.
  
  W: Forward<br/>
  A: Left<br/>
  S: Backward<br/>
  D: Right<br/>
  Anything else: Stop<br/>

  Speed is fixed and can be changed in the sketch under "int mainSpeed = 50;" which means 50% of the maximum speed.<br/>

Putty Setup:
  1. Download & Install Putty (https://www.putty.org/)
  2. Setup a Serial session with your COM port and at 115200 (default) speed
  3. Navigate in "Terminal" option and in "Line discipline options" then select "Local line editing: Force off"
  4. Alternatively you can set "Local echo: Force on" as well

![Alt Text](https://github.com/Lynxmotion/Rovers/blob/master/A4WD3/A4WD3_WASD/PuttyConfiguration.jpg)
