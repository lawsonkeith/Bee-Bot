![](https://github.com/lawsonkeith/Bee-Bot/blob/master/132___09/MAIN.JPG)


## Introduction

A simple wheeled robot that can be programmed to follow a course.  The robot has a keyboard that accepts movement commands and then executes them.  One unusual feature is that I've used a 6 DOF MEMS sensor to do all the navigation rather than wheel encoders.  The advantage here is that some a 6DOF IMU is a lot easier to integrate into any platform than wheel encoders, also slip isn't an issue.  The disadvantage is you don't know how far you've travelled.  I did hope it would be possible to do some linear vel/distance measurement with the IMU but unfortunately it's drift rate is way too high and the errors accumulate so it's only accurate for about 3s.

My son wanted to build one of these as he used a 'Bee Bot' at school, it also reminded me of the 'Big Trak' toys that were around in the 80s.  It's got PID control on heading so if you want to know how that works this is a good platform since quad copters and balancing robots tend to crash a lot when you get the constants wrong.  

I've included a sharp IR rangefinder as well to prevent the robot crashing into things, there's a speaker to tell you when you've pressed a button.

The idea is you program it to navigate round the room i.e send it to the kitchen and try and program it to come back etc.  You can watch a video of it doing some obstacle avoidance and simple navigation here:

http://youtu.be/1cztSxTJ8SI


## Hardware
Use the schematic to wire up your robot.  

![](https://github.com/lawsonkeith/Bee-Bot/blob/master/Schematic.png)


**Motors & Chassis** -
I bought a cheap robot platform off of ebay that came with wheels and motors for this project.

**Motor Control** -
A L9110 is used to proportionally control the two motors, I've added 2 capacitors to reduce noise also.  The motors can be controlled forwards or backwards.  The software scales the motors to be roughly linear and operate over their linear region only which is something like 20-80% duty cycle.

**Robot Control** -
This is done with an Arduino pro mini.  You need a 3.3V version or a version that can be 3.3/5V and then set it to 3.3V.  During development I had a programming lead permanently attached to the lid of the robot.  If you are thinking of using a 5V version with an LDO be carefull of the droput voltage, the ones on the chip are ultra low dropout which is useful when you load the batteries to maintain 3.3V.

**Command input** -
This comes from an analogue touch pad that has 5 buttons, each button outputs a different voltage into the Arduino analog input when pressed.

**Obstacle avoidance** -
This is done using a sharp 2Y0A21 F 39 IR range finder which feeds into the second Arduino analog input.

**Power** -
The unit is powered using 4xAAA hybrio batteries. This makes up the unregulated supply that feeds the motor control.  The rest of the devices are powered by a 3.3V regulated supply which is generated on the Arduino mini pro.  I made a little power distribution board that was fed via a switch to help me route the power on the device.  It's got a green LED on it to show when it's powered.  A rocker switch controls power.
I made this with some veroboard and 2.54mm Header strip:

VBatt   oxxxxxxo   
                  
V3.3V   oxxxxxxo
                 
GND     oxxxxxxo 
                  
Put the headers in line with the tracks and drill a few foles to feed the battery wires for strain relief.

**Audio** -
A speaker provides audio feedback on button presses and IMU status.

**Wiring** -
I've used 2.54mm headers and socket-socket connectors to wire everything up.

**Fixings**
I've used tie wraps and 3mm nuts and bolts to secure most things.  A electrical socket protector was used to make the range finder mount.  3mm washers were used where I needed to space things and Araldite where I needed to glue of fix something.

**IMU**
I use and MPU 6050 6 DOF accelerometer / gyro to make sure the robot maintained a heading.  The accelerometer doesn't really do anything.

![](https://github.com/lawsonkeith/Bee-Bot/blob/master/132___09/IMG_1123.JPG)

![](https://github.com/lawsonkeith/Bee-Bot/blob/master/132___09/IMG_1125.JPG)

![](https://github.com/lawsonkeith/Bee-Bot/blob/master/132___09/IMG_1122.JPG)

![](https://github.com/lawsonkeith/Bee-Bot/blob/master/132___09/IMG_1119.JPG)

![](https://github.com/lawsonkeith/Bee-Bot/blob/master/132___09/IMG_1120.JPG)

![](https://github.com/lawsonkeith/Bee-Bot/blob/master/132___09/IMG_1118.JPG)

# Software

The software is coded to do the following:

1. Wait for gyro heading to become stable (10s?) and tell the user with the speaker
2. Compile user commands (FWD, BACK, LEFT, RIGHT)
3. Execute commands as required (GO btn)
4. During execution use gyro to prevent drift due to non linearity of the motors
5. If an obstacle is detected in FWD mode disable the drives and issue a sound
6. If the 'GO' btn is held in delete current command list


**Note** - You need to install the I2cDev and MPU6050 Libraries in your Arduino->Libraries folder.  I've provided these as a zip file, unzip it and drag the folders to above location.



## Calibration - IMU
You need to calibrate the IMU, to do this use the RAW program 'MPU6050_raw.ino' I've included (not the examples).  The constants you get from there can be copied into your BBot.ino program.  You may get away with not doing it but I recommend that you do, my IMU performed mucho better once I'd calibrated it.

http://www.i2cdevlib.com/forums/topic/91-how-to-decide-gyro-and-accelerometer-offsett/


1. Put the MPU6050 in a flat and horizontal surface. Use an inclinometer to check that it is as horizontal as possible.
2. Modify the RAW program to put every offset to 0. ("setXGyroOffset/setYGyroOffset/setZGyroOffset/setZAccelOffset"  =0 ).
3. Upload the RAW program to your arduino and open serial monitor so you can see the values it is returning.
4. Leave it operating for a few minutes (5-10) so temperature gets stabilized.
5. Check the values in the serial monitor and write them down.
6. Now modify your program again updating your offsets and run the sketch, with updated offsets.
7. Repeat this process until your program is returning 0 for every gyro, 0 for X and Y accel, and +16384 for Z accel.
8. Once you achieve this, those are the offsets for that MPU6050, you will have to repeat this for every MPU6050 you use.


## Calibration - Motors
Unless you copy my setup you'll also need to calibrate the motors.  The motors are scaled to +/- 1000 where 0 just about gets them moving obviously the arduino PWM is generally 0-255 and unipolar.  1000 won't be 100% duty cycle though the all motors have a linear region, you'll have to find it for your robot by modifying the code to see how far the robot travels at certain demands in fixed time periods.  I made a table up and measured distance at 10% demand increments.  This provided me roughly with the following table where it can be seen there is a linear region.

![](https://github.com/lawsonkeith/Bee-Bot/blob/master/132___09/motor.JPG)

For me 20-80% duty cycle was linear, within that there has to be headroom for the PID to actually control robot heading.  If 1000% is full speed then you can't really do heading control anymore.  Also I wanted to guard against dipping the battery voltage when the robot travels forwards and resetting the Arduino.  So you can see in the above graph that I don't drive the motors 100% and I distribute the available speed between fore/aft and heading control.

It's worth spending a good hour making sure you've got the motors calibrated otherwise it'll be almost impossible to get the PID tuning right.  


## Calibration - PID heading control

The direction the robot faces is controlled by the IMU heading as derived from the gyro.  This ensures the robot travels in 90 degree arcs.  Without this the robot can't turn 90 degrees accurately or travel in a straight line.

Usually people use encoders for this but I'd not seen anyone try and use a gyro, this is actually common practice on sea going machines which is the industry I work in.

You may need to the alter the PID constants if yo change the motor or batteries. There's loads of info on how to do this on the web.  Basically P is proportional to the error and the I term gets rid of long term error.  You basically increase P as high as you can before the heading oscillates then tune I to get rid of long term errors when travelling in a straight line.

I found I needed 2 sets of constants as the moving case needed more agressive control than the stationary and turning case.



