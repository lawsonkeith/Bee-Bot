# Bee-Bot

A simple programmable robot that can be programmed to follow a course.  The robot has a keyboard that accepts movement commands and then executes them.  One unusual feature is that I've used a 6 DOF MEMS sensor to do all the navigation rather than wheel encoders.  The advantage here is that some a 6DOF IMU is a lot easier to integrate into any platform than wheel encoders.  My son wanted to build one of these as he used a 'Bee Bot' at school, it also reminded me of the 'Big Trak' toys that were around ing th 80s.

I've included a sharp IR rangefinder as well to prevent the robot crashing into things.


# Hardware
Use the schematic to wire up your robot.  I've included wheel encoders here but I didn't use them.  

![](https://github.com/lawsonkeith/Bee-Bot/blob/master/Schematic.png)


**Motors & Chassis** -
I bought a cheap robot platform off of ebay that came with wheels and motors for this project.

**Motor Control** -
A L9110 is used to proportionally control the two motors, I've added 2 capacitors to reduce noise also.  The motors can be controlled forwards or backwards.  The software scales the motorws to be roughly linear and operate over their linear region only which is something like 20-80% duty cycle.

**Robot Control** -
This is done with an arduino pro mini.  You need a 3.3V version or a version that can be 3.3/5V and then set it to 3.3V.  During development I had a programming lead permanently attached to the lid of the robot.  If you are thinking of using a 5V version with an LDO be carefull of the droput voltage, the ones on the chip are ultra low droput which is usefull when you load the batteries.

**Command input** -
This comes from an analogue touch pad that has 5 buttons, each button outputs a different voltage into the arduino analog input when pressed.

**Obstacle avoidance** -
This is done using a sharp IP range finder which feeds into the second aarduino analog input.

**Power** -
The unit is powered using 4xAAA hybrio batteries. This makes up the unregulated supply that feeds the motor control.  The rest of the devices are powered by a 3.3V regulated supply which is generated on the arduino mini pro.  I made a little power distribution board that was fed via a switch to help me route the power on the device.

**Audio ** -
A speaker provides audio feedback on button presses and IMU status.

**Wiring** -
I've used 2.54mm headers and socket-socket connectors to wire everything up.


# Software

The software is coded to do the following:

1. Wait for gyro heading to become stable (10s?) and tell the user with the speaker
2. Compile user commands (FWD, BACK, LEFT, RIGHT)
3. Execute commands as required (GO btn)
4. During execution use gyro to prevent drift due to non linearity of the motors
5. If an obstacle is detected in FWD mode disable the drives and issue a sound
6. If the 'GO' btn is held in delete current command list


Note - You need to install the I2cDev and MPU6050 Libraries in your Arduino->Libraries folder.  I've provided these as a zip file, unzip it and drag the folders to above location.





