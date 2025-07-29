// This file contains the main functionalities for the Mecanumbot
// Made by Balázs Nagy (2025)

// Includes
#include "MotorUtils.h"
#include <RC100.h>

// RC100 controller class and data init
RC100 rc;
int RcData = 0;

// Movement constants
int32_t lin_x = 0;
int32_t lin_y = 0;
int32_t ang_z = 0.0;

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
    lin_x = 0;
    lin_y = 0;
    ang_z = 0.0;

    // Movements
    if (RcData & RC100_BTN_U)    lin_x = 200;
    if (RcData & RC100_BTN_D)    lin_x = -200;
    if (RcData & RC100_BTN_2)    lin_y = -200;
    if (RcData & RC100_BTN_4)    lin_y = 200;
    if (RcData & RC100_BTN_6)    ang_z = 100;
    if (RcData & RC100_BTN_5)    ang_z = -100;

    // Grabber control
    if (RcData & RC100_BTN_R) grabber_status = CLOSED;   
    if (RcData & RC100_BTN_L) grabber_status = OPENED;   

    // Neck control
    if (RcData & RC100_BTN_1) neck_pos = NECK_MAX_POSITION_VALUE;
    if (RcData & RC100_BTN_3) neck_pos = 700; 
  }

  // Set Mecanum drive velocity parameters to each wheel (geometry should be involved)
  int32_t vFL = lin_x + lin_y - ang_z;
  int32_t vFR = lin_x - lin_y + ang_z;
  int32_t vBL = lin_x - lin_y - ang_z;
  int32_t vBR = lin_x + lin_y + ang_z;
  set_WheelVelocities(vBL, vBR, vFL, vFR);

  set_NeckPosition(neck_pos);
  set_GrabberStatus(grabber_status);
}
