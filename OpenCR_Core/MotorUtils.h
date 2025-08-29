#ifndef MOTOR_UTILS_H
#define MOTOR_UTILS_H

// DynamixelSDK needs to be installed first
#include <DynamixelSDK.h>

// General parameters for all motors
#define DEVICENAME               "OpenCR_DXL_Port"  // For OpenCR
#define BAUDRATE                 115200
#define VOLTAGE_PIN              16                 // ADC0 on OpenCR

#define TORQUE_ENABLE            1
#define TORQUE_DISABLE           0

#define ID_NECK                  7
#define ID_GRABBER_L             6
#define ID_GRABBER_R             5
#define ID_WHEEL_FR              4
#define ID_WHEEL_FL              3
#define ID_WHEEL_BR              2
#define ID_WHEEL_BL              1

#define OPENED                   1
#define CLOSED                   0 

// Mechanum drive working parameters
#define WHEEL_SEPARATION_X       1 // 13 cm
#define WHEEL_SEPARATION_Y       0 // 30,5 cm
#define WHEEL_RADIUS             1 // 6,5 cm

// AX-12A motor parameters (Neck and Grabber) - Position control
// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/

#define AX_ADDR_TORQUE_ENABLE        24 // RW
#define AX_ADDR_GOAL_POSITION        30 // RW
#define AX_ADDR_MOVING_SPEED         32 // RW
#define AX_ADDR_PRESENT_POSITION     36 // R
#define AX_ADDR_PRESENT_SPEED        38 // R
#define AX_ADDR_PRESENT_VOLTAGE      42 // R
#define AX_ADDR_PRESENT_TEMPERATURE  43 // R

#define AX_PROTOCOL_VERSION          1.0

#define NECK_MIN_POSITION_VALUE      200  
#define NECK_MAX_POSITION_VALUE      860  
#define GRABBER_MIN_POSITION_VALUE   160
#define GRABBER_FRONT_POSITION_VALUE 512  
#define GRABBER_MAX_POSITION_VALUE   854    

// XM430-W210-T motor parameteres (Wheels) - Velocity control
// https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/

#define XM_ADDR_OPERATING_MODE      11  // RW 
#define XM_ADDR_VELOCITY_LIMIT      44  // RW 
#define XM_ADDR_TORQUE_ENABLE       64  // RW
#define XM_ADDR_GOAL_VELOCITY       104 // RW
#define XM_ADDR_PRESENT_VELOCITY    128 // R
#define XM_ADDR_PRESENT_TEMPERATURE 128 // R


#define XM_PROTOCOL_VERSION         2.0

#define XM_VELOCITY_VALUE           200 
#define XM_LEN_GOAL_VELOCITY        4
#define XM_NUM_MOTORS               4

#define XM_VELOCITY_MODE            1

extern dynamixel::PortHandler    *portHandler;
extern dynamixel::PacketHandler  *packetHandlerAX;
extern dynamixel::PacketHandler  *packetHandlerXM;
extern dynamixel::GroupSyncWrite *groupSyncWrite;

// Init functions
void init_Dynamixel(const char* device_name, uint32_t baudrate);
void init_Neck();
void init_Grabber();
void init_Wheel(int ID);
void init_AllWheels();

// Set functions
void set_TorqueEnableAX(int ID);

void set_NeckPosition(uint16_t position);
void set_GrabberStatus(int grabber_status);

void set_WheelVelocities(int32_t vBL, int32_t vBR, int32_t vFL, int32_t vFR);

// Byte functions
void writeByte(dynamixel::PacketHandler* handler, int ID, int ADDRESS, uint16_t DATA);

#endif
