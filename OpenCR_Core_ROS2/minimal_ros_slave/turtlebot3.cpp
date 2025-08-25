// Minimal double-slave ROS architecture example for OpenCR
// - Exposes two DYNAMIXEL slaves over Serial to act as ROS2-accessible devices
// - Slave A (AX): neck + grabber (position / torque)
// - Slave X (XM): 4 wheel velocities (velocity / torque)
// Minimal: only handles writes from master and calls MotorUtils.* control helpers

#include "turtlebot3.h"

#ifndef DXL_ERR_OK
#define DXL_ERR_OK 0
#endif
#ifndef DXL_ERR_ACCESS
#define DXL_ERR_ACCESS 1
#endif

#define SERIAL_DXL_SLAVE Serial
#define ID_AX_SLAVE 201
#define ID_XM_SLAVE 202
#define MODEL_NUM_AX_SLAVE 0x5001
#define MODEL_NUM_XM_SLAVE 0x5002
#define FIRMWARE_VER 1

// Simple control storage for the AX slave - neck and grabber
static struct {
  uint8_t torque_enable;
  uint16_t neck_goal;
  uint8_t grabber_goal; // treat as OPENED/CLOSED
} control_ax;

// Simple control storage for the XM slave - 4 wheel velocities
static struct {
  uint8_t torque_enable;
  int32_t goal_vel_bl;
  int32_t goal_vel_br;
  int32_t goal_vel_fl;
  int32_t goal_vel_fr;
} control_xm;

Dynamixel2Arduino dxl(Serial);
DYNAMIXEL::USBSerialPortHandler port_dxl_slave(SERIAL_DXL_SLAVE);
DYNAMIXEL::Slave dxl_slave_ax(port_dxl_slave, MODEL_NUM_AX_SLAVE);
DYNAMIXEL::Slave dxl_slave_xm(port_dxl_slave, MODEL_NUM_XM_SLAVE);

// Addresses (small, minimal)
enum AX_ADDRS {
  AX_ADDR_INFO = 2,
  AX_ADDR_TORQUE = 10,
  AX_ADDR_NECK_GOAL = 12,
  AX_ADDR_GRABBER_GOAL = 14,
};

enum XM_ADDRS {
  XM_ADDR_INFO = 2,
  XM_ADDR_TORQUE = 20,
  XM_ADDR_GOAL_VEL_BL = 22, // 4 bytes
  XM_ADDR_GOAL_VEL_BR = 26,
  XM_ADDR_GOAL_VEL_FL = 30,
  XM_ADDR_GOAL_VEL_FR = 34,
};

// Forward declarations
static void dxl_slave_ax_write_callback(uint16_t item_addr, uint8_t &dxl_err_code, void* arg);
static void dxl_slave_xm_write_callback(uint16_t item_addr, uint8_t &dxl_err_code, void* arg);

// One-time boot sequence flag
static bool boot_sequence_done = false;

// Perform a safe boot choreography: enable torques and move neck, grabber and wheels briefly
static void performBootSequence() {
  if (boot_sequence_done) return;
  Serial.println("Boot sequence start...");

  // Ensure AX torque for neck and grabbers
  set_TorqueEnableAX(ID_NECK);
  set_TorqueEnableAX(ID_GRABBER_L);
  set_TorqueEnableAX(ID_GRABBER_R);
  delay(200);

  // Neck sweep: move to max, then min, then center
  Serial.println("Neck: to max");
  set_NeckPosition(NECK_MAX_POSITION_VALUE);
  delay(700);
  Serial.println("Neck: to min");
  set_NeckPosition(NECK_MIN_POSITION_VALUE);
  delay(700);
  Serial.println("Neck: to center");
  set_NeckPosition((NECK_MIN_POSITION_VALUE + NECK_MAX_POSITION_VALUE)/2);
  delay(500);

  // Grabber open/close
  Serial.println("Grabber: open");
  set_GrabberStatus(OPENED);
  delay(500);
  Serial.println("Grabber: close");
  set_GrabberStatus(CLOSED);
  delay(500);
  set_GrabberStatus(OPENED);
  delay(300);

  // Wheels: small forward pulse and stop
  Serial.println("Wheels: small forward pulse");
  set_WheelVelocities(100, 100, 100, 100); // small test velocity
  delay(600);
  set_WheelVelocities(0,0,0,0);
  delay(200);

  Serial.println("Boot sequence done");
  boot_sequence_done = true;
}

// Initialize everything and start the slaves
void TurtleBot3Core::begin() {
  // Serial for slave protocol
  Serial.begin(57600);

  // Initialize motors (MotorUtils.cpp expected in project)
  init_Dynamixel(DEVICENAME, BAUDRATE);
  init_Neck();
  init_Grabber();
  init_AllWheels();

  // Initialize control storage
  control_ax.torque_enable = 0;
  control_ax.neck_goal = 512;
  control_ax.grabber_goal = OPENED;

  control_xm.torque_enable = 0;
  control_xm.goal_vel_bl = 0;
  control_xm.goal_vel_br = 0;
  control_xm.goal_vel_fl = 0;
  control_xm.goal_vel_fr = 0;

  // Begin two slaves
  dxl_slave_ax.begin();
  dxl_slave_ax.setPortProtocolVersion(2.0);
  dxl_slave_ax.setFirmwareVersion(FIRMWARE_VER);
  dxl_slave_ax.setID(ID_AX_SLAVE);

  dxl_slave_xm.begin();
  dxl_slave_xm.setPortProtocolVersion(2.0);
  dxl_slave_xm.setFirmwareVersion(FIRMWARE_VER);
  dxl_slave_xm.setID(ID_XM_SLAVE);

  // Add minimal control items (reference the variables above)
  dxl_slave_ax.addControlItem(AX_ADDR_TORQUE, control_ax.torque_enable);
  dxl_slave_ax.addControlItem(AX_ADDR_NECK_GOAL, control_ax.neck_goal);
  dxl_slave_ax.addControlItem(AX_ADDR_GRABBER_GOAL, control_ax.grabber_goal);

  dxl_slave_xm.addControlItem(XM_ADDR_TORQUE, control_xm.torque_enable);
  dxl_slave_xm.addControlItem(XM_ADDR_GOAL_VEL_BL, control_xm.goal_vel_bl);
  dxl_slave_xm.addControlItem(XM_ADDR_GOAL_VEL_BR, control_xm.goal_vel_br);
  dxl_slave_xm.addControlItem(XM_ADDR_GOAL_VEL_FL, control_xm.goal_vel_fl);
  dxl_slave_xm.addControlItem(XM_ADDR_GOAL_VEL_FR, control_xm.goal_vel_fr);

  // Register write callbacks
  dxl_slave_ax.setWriteCallbackFunc(dxl_slave_ax_write_callback);
  dxl_slave_xm.setWriteCallbackFunc(dxl_slave_xm_write_callback);

  // Run boot sequence so robot moves through a short choreography on startup
  performBootSequence();
}

// Main periodic processing: let slaves handle incoming packets
void TurtleBot3Core::run() {
  // Let slaves process incoming packets from ROS master
  dxl_slave_ax.processPacket();
  dxl_slave_xm.processPacket();

  // No periodic feedback implemented in this minimal example (reads omitted)
  delay(2);
}

// AX slave write: map writes into MotorUtils
static void dxl_slave_ax_write_callback(uint16_t item_addr, uint8_t &dxl_err_code, void* arg) {
  (void)arg;
  dxl_err_code = DXL_ERR_OK;

  switch(item_addr) {
    case AX_ADDR_TORQUE:
      if (control_ax.torque_enable)
        set_TorqueEnableAX(ID_NECK);
      else
        set_TorqueEnableAX(ID_NECK); // minimal: same call, real implementation should disable
      break;

    case AX_ADDR_NECK_GOAL:
      // enforce some limits very simply
      if (control_ax.neck_goal < NECK_MIN_POSITION_VALUE) control_ax.neck_goal = NECK_MIN_POSITION_VALUE;
      if (control_ax.neck_goal > NECK_MAX_POSITION_VALUE) control_ax.neck_goal = NECK_MAX_POSITION_VALUE;
      set_NeckPosition(control_ax.neck_goal);
      break;

    case AX_ADDR_GRABBER_GOAL:
      if (control_ax.grabber_goal != OPENED) control_ax.grabber_goal = CLOSED;
      set_GrabberStatus(control_ax.grabber_goal);
      break;

    default:
      dxl_err_code = DXL_ERR_ACCESS;
      break;
  }
}

// XM slave write: map velocities into MotorUtils
static void dxl_slave_xm_write_callback(uint16_t item_addr, uint8_t &dxl_err_code, void* arg) {
  (void)arg;
  dxl_err_code = DXL_ERR_OK;

  switch(item_addr) {
    case XM_ADDR_TORQUE:
      // minimal - no dedicated torque call for XM in header; leave as noop
      break;

    case XM_ADDR_GOAL_VEL_BL:
    case XM_ADDR_GOAL_VEL_BR:
    case XM_ADDR_GOAL_VEL_FL:
    case XM_ADDR_GOAL_VEL_FR:
      // whenever any wheel velocity is updated, push all four values to the motors
      set_WheelVelocities(control_xm.goal_vel_bl, control_xm.goal_vel_br, control_xm.goal_vel_fl, control_xm.goal_vel_fr);
      break;

    default:
      dxl_err_code = DXL_ERR_ACCESS;
      break;
  }
}
