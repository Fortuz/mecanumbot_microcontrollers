# Mecanumbot Microcontrollers

This repository contains the microcontroller codes for the Mecanumbot project.
The Mecanumbot project is a small, modified Turtlebot3 robot. Utilizes an OpenCR for motor controll. Equipped with 4 mecanum wheels, a grabber containing 2 motors and a camera moving neck motor.
The second microcontroller is an Arduino Nano which controlls the LEDs on the robot.
The robot can be controlled with the RC100 controller.

## Authors
**Balázs Nagy** <br> 
**Benedek Fegyó** <br>
**Csenge Hubay** <br>

## Project outline

- **Nano_LED**: Code for the Nano micro controller to controll the LEDs on the robot. <br>
- **OpenCR_Core**: Code for a minimal RC100 remote control for the Mecanumbot. (No ROS2 included) <br>
- **turtlebot3_mecanumbot**: Modified version of the original Turtlebot3 microcontroller code to accomodate the Mecanumbot extended functionality (Not tested yet)

## Requirements
Dynamixel Workbench (Windows only) <br>
Arduino IDE <br>
Dynamixel SDK <br>
RC100 <br>
FastLED <br>

## Setup
Navigate into the Arduino sketch folder and glone this repository. <br>
(Or clone it anywhere and open it in Arduino IDE)

```bash
$ cd  ~/Documents/Arduino
$ git clone https://github.com/Fortuz/mecanumbot_microcontrollers.git
```
The Nano_LED part can be uploaded to the Arduino Nano. Set the appropriate port (COM6), board (Arduino Nano), and bootloader (ATmega328P) in the Arduino IDE and push upload. <br>
The OpenCR_Core part can be uploaded to the OpenCR board. Set the appropriate port (COM11), board (OpenCR Board), and  in the Arduino IDE and push upload.

## Mechanical Assembly

The base is a modified Turtlebot3 Waffle. The [documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) of the Turtlebots can be found here.
The robot is mounted with extra motors and custom made 3D printed elements.

## Motor setup

|  | Neck | Grabber L | Grabber R | Wheel FL | Wheel FR | Wheel BR | Wheel BL |
|:--- |:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Type | AX-12A | AX-12A | AX-12A | XM430-W210-T | XM430-W210-T | XM430-W210-T | XM430-W210-T |
| ID | 7 | 6 | 5 | 3 | 4 | 2 | 1 |
| Protocol | 1 | 1 | 1 | 2 | 2 | 2 | 2 |
| Baudrate | 1000000 | 1000000 | 1000000 | 1000000 | 1000000 | 1000000 | 1000000|
| Mode | Position | Position | Position | Velocity | Velocity | Velocity | Velocity |

[XM430-W210-T motor datasheet](https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/) <br>
[AX-12A motor datasheet](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/) <br>

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot_microcontrollers/blob/main/docs/images/Mecanumbot_MotorIDs.png" width="600" alt="Mecanumbot">
</p>

## Project details

### General working of the code

There is a control table, that bounds the memory adresses to the motors and certain functionalities. There are two sets of motors as it can be seen in the above diagram. The AX motors work on protocol 1 and the XM motors work on protocol 2. This means that protocol switching is implemented in the code, i.e. the base protocol is 2 and when an AX motor is being communicated with, the protocol changes to 1.

### Nano_LED

The Nano is a small micro controller, microros is not an option. The Nano just uses Serial communication. 

Commands:
"GET" - The Nano responses with a string encoding the current state of the leds
"0102010201020102" - Sets the encoded mode and color combination on the leds

String assembly - Every mode and every color is 2 character with the following structure

[Mode_FL, Color_FL, Mode_FR, Color_FR,, Mode_BR, Color_BR, Mode_BL, Color_BL] 

| Modes | Value |
|:--- |:---:|
| WAVE_RIGHT | 01 |
| WAVE_LEFT  | 02 |
| PULSE      | 03 |
| SOLID      | 04 |

| Colors | Value |
|:--- |:---:|
| BLACK   | 01 |
| WHITE   | 01 |
| GREEN   | 02 |
| RED     | 03 |
| BLUE    | 04 |
| CYAN    | 05 |
| PINK    | 06 |
| YELLOW | 07 |

### OpenCR_Core

The layout of the robot control buttons on the RC100 controller can be seen below.

<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot_microcontrollers/blob/main/docs/images/RC100_button_layout.png" width="600" alt="Mecanumbot">
</p>

### OpenCR_Core_ROS2

A minimal working example to use ROS2 in the communication and test out different functionalities.

### turtlebot3_mecanumbot

In the same architecture as the turtlebot3 provides the functionality of a mecanum wheeled robot. The mecanumbot also mounted with 2 gripper motors and a neck motor. The gripper and neck motors (AX) are different from the driving motors (XM) which means an additional protol was needed to control them. The extra control table items and control functions are implemented in it. 
