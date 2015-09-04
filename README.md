# Bee-Bot

A simple programmable robot that can be programmed to follow a course.  The robot has a keyboard that accepts movement commands and then executes them.  One unusual feature is that I've used a 6 DOF MEMS sensor to do all the navigation rather than encoders.

# Hardware

**Motors & Chassis**
I bought a cheap robot platform off of ebay that came with wheels and motors for this project.

**Motor Control**
A L9110 is used to proportionally control the two motors, I've added 2 capacitors to reduce noise also.  The motors can be controlled forwards or backwards.  The software scales the motorws to be roughly linear and operate over their linear region only which is something like 20-80% duty cycle.

**Robot Control**
This is done with an arduino pro mini.  You need a 3.3V version or a version that can be 3.3/5V and then set it to 3.3V.  During development I had a programming lead permanently attached to the lid of the robot.

**Command input**
This comes from an analogue touch pad that has 5 buttons, each button outputs a different voltage into the arduino analog input when pressed.

**Obstacle avoidance**
This is done using a sharp IP range finder which feeds into the second aarduino analog input.

**Power**
The unit is powered using 4xAAA hybrio batteries. This makes up the unregulated supply that feeds the motor control.  The rest of the devices are powered by a 3.3V regulated supply which is generated on the arduino mini pro.  I made a little power distribution board that was fed via a switch to help me route the power on the device.

**Audio **
A speaker provides audio feedback on button presses and IMU status.


# Software

The software is coded to do the following:

(1) Wait for gyro heading to become stable
(2) Compile user commands
(3) Execute commands as required
(4) During execution us gyro to prevent drift
(5) If an obstacle is detected in FWD mode disable the drives and issue a sound

