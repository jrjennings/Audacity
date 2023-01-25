# Audacity
## a Robo-Magellan variant of the Sawppy rover

Build a variant of the Sawppy Rover using the same basic chassis as the original build by Rodger Cheng see: https://github.com/Roger-random/Sawppy_Rover in order to compete in the Seattle Robotics Society Robo-Magellan see: https://robothon.org/rules-robo-magellan/

Make the following modifications to original design:
<ul>
<li>change to gear motors with encoders instead of serial bus servos for drive motors.
<li>upgrade the stearing servos to Dynamixels instead of LewanSoul serial bus servos
<li>add an android cell phone for its sensors (eg. accelerometer, compass, GPS) in order to send movement commands to Arduino Mega on the rover via Bluetooth.
<li>add machine vision to locate orange cone and navigate towards it
<li>add ultrasonic sensors for obstical avoidance
<li>add a bumper touch sensor
<li>add tire to wheel for durability and more grip on smooth surfaces
</ul>
