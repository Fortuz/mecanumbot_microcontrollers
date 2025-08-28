# Mecanumbot Microcontrollers

This repository contains the microcontroller codes for the Mecanumbot project.
The Mecanumbot project is a small, modified Turtlebot3 robot. Utilizes an OpenCR for motor controll. Equipped with 4 mecanum wheels, a grabber containing 2 motors and a camera moving neck motor.
The second microcontroller is an Arduino Nano which controlls the LEDs on the robot.

Functionality: the robot can be controlled with the RC100 controller.

Layout for the controller: <br>
U - Go Forward <br>
D - Go Backward <br>
R - Close Grippers <br>
L - Open Grippers <br>
1 - Look up with camera <br>
2 - Go Left <br>
3 - Look down with camera <br>
4 - Go Right <br>
5 - Turn Right <br>
6 - Turn Left <br>

The LEDs just set to a default motion and will lit up when inicialized. Not controlled directly with the Controler but the functions are implemented.

## TODO

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot_microcontrollers/blob/main/docs/images/Mecanumbot_MotorIDs.png" width="600" alt="Mecanumbot">
</p>

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot_microcontrollers/blob/main/docs/images/RC100_button_layout.png" width="600" alt="Mecanumbot">
</p>


## Requirements:
Dynamixel Workbench (Windows only) <br>
Arduino IDE <br>
Dynamixel SDK <br>
RC100 <br>
FastLED <br>

## Setup 
Tested and Recommended on Windows <br>
(Using Dynamixel Workbench to set the motor parameters is way too easy...)

### Mechanical Assembly

The base is a modified Turtlebot3 Waffle. The [documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) of the Turtlebots can be found here.
The robot is mounted with extra motors and custom made 3D printed elements.

### Motor setup

### Microcontroller setup

Navigate into the Arduino sketch folder and glone this repository. <br>
(Or clone it anywhere and open it in Arduino IDE)

```bash
$ cd  ~/Documents/Arduino
$ git clone https://github.com/Fortuz/mecanumbot_microcontrollers.git
```
The Nano_LED part can be uploaded to the Arduino Nano. Set the appropriate port (COM6), board (Arduino Nano), and bootloader (ATmega328P) in the Arduino IDE and push upload. <br>
The OpenCR_Core part can be uploaded to the OpenCR board. Set the appropriate port (COM11), board (OpenCR Board), and  in the Arduino IDE and push upload.
