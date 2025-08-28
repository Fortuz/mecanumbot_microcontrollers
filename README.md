# Mecanumbot Microcontrollers

This repository contains the microcontroller codes for the Mecanumbot project.
The Mecanumbot project is a small, modified Turtlebot3 robot. Utilizes an OpenCR for motor controll. Equipped with 4 mecanum wheels, a grabber containing 2 motors and a camera moving neck motor.
The second microcontroller is an Arduino Nano which controlls the LEDs on the robot.
The robot can be controlled with the RC100 controller.

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
|--- |--- |--- |--- |--- |--- |--- |--- |
| Type | AX-12A | AX-12A | AX-12A | XM430-W210-T | XM430-W210-T | XM430-W210-T | XM430-W210-T |
| ID | 7 |6 |5 |3 |4 |2 |1 |
| Protocol | 1 | 1 | 1 | 2 | 2 | 2 | 2 |
| Baudrate | 1 000 000 | 1 000 000 | 1 000 000 | 1 000 000 | 1 000 000 | 1 000 000 | 1 000 000 |
| Mode | Position | Position | Position | Velocity | Velocity | Velocity | Velocity |


<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot_microcontrollers/blob/main/docs/images/Mecanumbot_MotorIDs.png" width="600" alt="Mecanumbot">
</p>


<p align="center">
  <img src="https://github.com/Fortuz/mecanumbot_microcontrollers/blob/main/docs/images/RC100_button_layout.png" width="600" alt="Mecanumbot">
</p>


