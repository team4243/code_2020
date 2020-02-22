# Testing Priorities:<br/><br/>

**__Zero:__** <br/>
-Check __Motor Controllers__, CAN Bus Device Numbers, is firmware updated <br/>
-Can we communicate wirelessly <br/><br/>

**__One:__** <br/>
-*cameras yes* <br/>
-Check if robot __drives/strafes/rotates__ correctly <br/>
-be able to make sure we can move a certain amount in autonomous <br/>
-Can we toggle between __high-low gear?__ <br/>
-*Check if field orientation works correctly IFF gyro works AND we are using it* <br/>
-does Gyro __zero__ if we are using it? <br/><br/>


**__Two:__** <br/>
-Do __lead screws__ rotate in the right direction (corresponding to joystick direction) <br/>
  -Can they rotate in both directions? <br/>
-Do we get angle from gyro <br/>
-Do we get __encoders values__ from both arms <br/>
-Check spike in current? <br/>
-*auto-stop if current spike* <br/>
-Can auto-lift work <br/><br/>


**__Three:__** <br/>
-See if __Control Panel__ motor spins <br/>
-Check if we can detect colour <br/>
  -Check color changes <br/>
  -Check num of rotations <br/>
-Check if we can automatically turn 3 times/to a colour <br/>
