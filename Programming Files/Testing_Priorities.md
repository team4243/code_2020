# Testing Priorities:

**__Zero:__**
-Check __Motor Controllers__, CAN Bus Device Numbers, is firmware updated
-Can we communicate wirelessly


**__One:__**
-*cameras yes*
-Check if robot __drives/strafes/rotates__ correctly
-be able to make sure we can move a certain amount in autonomous
-Can we toggle between __high-low gear?__
-*Check if field orientation works correctly IFF gyro works AND we are using it*
-does Gyro __zero__ if we are using it?


**__Two:__**
-Do __lead screws__ rotate in the right direction (corresponding to joystick direction)
  -Can they rotate in both directions?
-Do we get angle from gyro
-Do we get __encoders values__ from both arms
-Check spike in current?
-*auto-stop if current spike*
-Can auto-lift work


**__Three:__**
-See if __Control Panel__ motor spins
-Check if we can detect colour
  -Check color changes
  -Check num of rotations
-Check if we can automatically turn 3 times/to a colour