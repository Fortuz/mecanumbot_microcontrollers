// This file contains the main functionalities for the Mecanumbot
// Made by Balázs Nagy (2025)

// Test 

// Includes
#include "MotorUtils.h"
#include <RC100.h>

// RC100 controller class and data init
RC100 rc;
int RcData = 0;

// Movement constants
int32_t goal_linear_x_velocity = 0;
int32_t goal_linear_y_velocity = 0;
int32_t goal_angular_velocity = 0.0;

// Position targets for grabber and neck
int grabber_L_pos = 512;
int grabber_R_pos = 512;
int neck_pos = 800;
int grabber_status = 2;

void setup() 
{
  // Init communication and controller
  Serial.begin(BAUDRATE);
  rc.begin(1);
  Serial.println("RC100 Controller is ready!");

  init_Dynamixel(DEVICENAME, BAUDRATE);
  init_Neck();
  init_Grabber();

  // InitWheels();
  init_AllWheels();
  Serial.println("Wheels are ready!");
}

void loop() {
  // Read the Controller data
  if (rc.available()) {
    uint16_t RcData = rc.readData();
    goal_linear_x_velocity = 0;
    goal_linear_y_velocity = 0;
    goal_angular_velocity = 0.0;

    // Movements
    if (RcData & RC100_BTN_U)    goal_linear_x_velocity =  200;
    if (RcData & RC100_BTN_D)    goal_linear_x_velocity = -200;
    if (RcData & RC100_BTN_2)    goal_linear_y_velocity = -200;
    if (RcData & RC100_BTN_4)    goal_linear_y_velocity =  200;
    if (RcData & RC100_BTN_6)    goal_angular_velocity  =  100;
    if (RcData & RC100_BTN_5)    goal_angular_velocity  = -100;

    // Grabber control
    if (RcData & RC100_BTN_R) grabber_status = CLOSED;   
    if (RcData & RC100_BTN_L) grabber_status = OPENED;   

    // Neck control
    if (RcData & RC100_BTN_1) neck_pos = NECK_MAX_POSITION_VALUE;
    if (RcData & RC100_BTN_3) neck_pos = 700; 
  }

  // Set Mecanum drive velocity parameters to each wheel (x is straight, y is strafe)
  // TIP: Balance the robot to keep a low strafe misalignment
  int32_t velocity_FL = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  int32_t velocity_FR = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  int32_t velocity_BL = (1/WHEEL_RADIUS) * (goal_linear_x_velocity - goal_linear_y_velocity - (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);
  int32_t velocity_BR = (1/WHEEL_RADIUS) * (goal_linear_x_velocity + goal_linear_y_velocity + (WHEEL_SEPARATION_X + WHEEL_SEPARATION_Y) * goal_angular_velocity);

  set_WheelVelocities(velocity_BL, velocity_BR, velocity_FL, velocity_FR);

  set_NeckPosition(neck_pos);
  set_GrabberStatus(grabber_status);

  // Read Voltage
  int adc_value = analogRead(BDPIN_BAT_PWR_ADC);
  float vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value/100;

}
