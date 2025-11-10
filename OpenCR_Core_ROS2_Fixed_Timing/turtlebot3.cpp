/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "turtlebot3.h"
#include "camera_gimbal_driver.h"

/*******************************************************************************
* Definition of dependency data according to TB3 model.
*******************************************************************************/
typedef struct TB3ModelInfo{
  const char* model_str;
  uint32_t model_info;
  float wheel_radius;
  float wheel_separation_x;
  float wheel_separation_y;
  float turning_radius;
  float robot_radius;
  bool has_manipulator;
} TB3ModelInfo;

static const TB3ModelInfo mecanumbot_info = {
  "Mecanumbot",
  4,
  0.033,
  0.287,
  0.05,  // TODO tune this parameter
  0.001,
  0.220,
  false,
};

/*******************************************************************************
* Declaration for motors
*******************************************************************************/
static Turtlebot3MotorDriver motor_driver;
// static OpenManipulatorDriver manipulator_driver(motor_driver.getDxl());
static CameraGimbalDriver ax_driver(motor_driver.getDxl());

static const TB3ModelInfo* p_tb3_model_info;
static float max_linear_velocity, min_linear_velocity;
static float max_angular_velocity, min_angular_velocity;

static float goal_velocity[VelocityType::TYPE_NUM_MAX] = {0.0, 0.0, 0.0}; // lin_x, lin_y, ang_z
static float goal_velocity_from_cmd[VelocityType::TYPE_NUM_MAX] = {0.0, 0.0, 0.0}; 
static float goal_velocity_from_rc100[VelocityType::TYPE_NUM_MAX] = {0.0, 0.0, 0.0};  
static float goal_velocity_from_button[VelocityType::TYPE_NUM_MAX] = {0.0, 0.0, 0.0};

static bool goal_state_from_rc100[2] = {0, 0};

static void update_goal_velocity_from_3values(void);
static void test_motors_with_buttons(uint8_t buttons);
static bool get_connection_state_with_motors();
static void set_connection_state_with_motors(bool is_connected);
static bool get_connection_state_with_joints();
static void set_connection_state_with_joints(bool is_connected);

/*******************************************************************************
* Declaration for sensors
*******************************************************************************/
static Turtlebot3Sensor sensors;

/*******************************************************************************
* Declaration for diagnosis
*******************************************************************************/
static Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
static Turtlebot3Controller controllers;

/*******************************************************************************
* Declaration for DYNAMIXEL Slave Function
*******************************************************************************/
#define SERIAL_DXL_SLAVE Serial
const uint8_t ID_DXL_SLAVE = 200;              // Open CR ID
const uint16_t MODEL_NUM_DXL_SLAVE = 0x5000;   // Open CR Model Number
const float PROTOCOL_VERSION_DXL_SLAVE = 2.0;
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;
// Poll AX (Protocol 1) present positions less frequently to avoid blocking
// the main loop if occasional timeouts occur on the bus.
const uint32_t AX_UPDATE_INTERVAL_MS = 100; // ms

static void dxl_slave_write_callback_func(uint16_t addr, uint8_t &dxl_err_code, void* arg); // Which protocol?

static bool get_connection_state_with_ros2_node();
static void set_connection_state_with_ros2_node(bool is_connected);
static void update_connection_state_with_ros2_node();                    // TODO

static void update_imu(uint32_t interval_ms);
static void update_times(uint32_t interval_ms);
static void update_gpios(uint32_t interval_ms);
static void update_motor_status(uint32_t interval_ms);
static void update_ax_motor_status(uint32_t interval_ms);
static void update_battery_status(uint32_t interval_ms);
static void update_analog_sensors(uint32_t interval_ms);
//static void update_joint_status(uint32_t interval_ms);

// Writers scheduled per-protocol windows (20 ms each)
static void process_ax_writes(uint32_t interval_ms);
static void process_wheel_writes(uint32_t interval_ms);

DYNAMIXEL::USBSerialPortHandler port_dxl_slave(SERIAL_DXL_SLAVE);
DYNAMIXEL::Slave dxl_slave(port_dxl_slave, MODEL_NUM_DXL_SLAVE);
//DYNAMIXEL::Slave dxl_slave_ax(port_dxl_slave, MODEL_NUM_AX_SLAVE);
//DYNAMIXEL::Slave dxl_slave_xm(port_dxl_slave, MODEL_NUM_XM_SLAVE);

enum ControlTableItemAddr{
  ADDR_MODEL_INFORM    = 2,


  ADDR_MILLIS          = 10,
  
  ADDR_DEBUG_MODE      = 14,
  ADDR_CONNECT_ROS2    = 15,
  ADDR_CONNECT_MANIP   = 16,

  ADDR_DEVICE_STATUS   = 18,
  ADDR_HEARTBEAT       = 19,

  ADDR_USER_LED_1      = 20,
  ADDR_USER_LED_2      = 21,
  ADDR_USER_LED_3      = 22,
  ADDR_USER_LED_4      = 23,

  ADDR_BUTTON_1        = 26,
  ADDR_BUTTON_2        = 27,
  ADDR_BUMPER_1        = 28,
  ADDR_BUMPER_2        = 29,

  ADDR_ILLUMINATION    = 30,
  ADDR_IR              = 34,
  ADDR_SORNA           = 38,

  ADDR_BATTERY_VOLTAGE = 42,
  ADDR_BATTERY_PERCENT = 46,

  ADDR_SOUND           = 50,

  ADDR_IMU_RECALIBRATION  = 59,
  ADDR_ANGULAR_VELOCITY_X = 60,
  ADDR_ANGULAR_VELOCITY_Y = 64,
  ADDR_ANGULAR_VELOCITY_Z = 68,
  ADDR_LINEAR_ACC_X       = 72,
  ADDR_LINEAR_ACC_Y       = 76,
  ADDR_LINEAR_ACC_Z       = 80,
  ADDR_MAGNETIC_X         = 84,
  ADDR_MAGNETIC_Y         = 88,
  ADDR_MAGNETIC_Z         = 92,
  ADDR_ORIENTATION_W      = 96,
  ADDR_ORIENTATION_X      = 100,
  ADDR_ORIENTATION_Y      = 104,
  ADDR_ORIENTATION_Z      = 108,

  ADDR_PRESENT_CURRENT_BL  = 120,
  ADDR_PRESENT_CURRENT_BR  = 124,
  ADDR_PRESENT_CURRENT_FL  = 128,
  ADDR_PRESENT_CURRENT_FR  = 132,
  ADDR_PRESENT_VELOCITY_BL = 136,
  ADDR_PRESENT_VELOCITY_BR = 140,
  ADDR_PRESENT_VELOCITY_FL = 144,
  ADDR_PRESENT_VELOCITY_FR = 148,
  ADDR_PRESENT_POSITION_BL = 152,
  ADDR_PRESENT_POSITION_BR = 156,
  ADDR_PRESENT_POSITION_FL = 160,
  ADDR_PRESENT_POSITION_FR = 164,

  ADDR_MOTOR_CONNECT      = 168,
  ADDR_MOTOR_TORQUE       = 169,
  ADDR_CMD_VEL_LINEAR_X   = 170,
  ADDR_CMD_VEL_LINEAR_Y   = 174,
  ADDR_CMD_VEL_LINEAR_Z   = 178,
  ADDR_CMD_VEL_ANGULAR_X  = 182,
  ADDR_CMD_VEL_ANGULAR_Y  = 186,
  ADDR_CMD_VEL_ANGULAR_Z  = 190,
  ADDR_PROFILE_ACC_BL      = 194,
  ADDR_PROFILE_ACC_BR      = 198,
  ADDR_PROFILE_ACC_FL      = 202,
  ADDR_PROFILE_ACC_FR      = 206,
  

  // AX motors for manipulator and camera
  AX_ADDR_TORQUE = 210,
  AX_ADDR_NECK_GOAL = 214,
  AX_ADDR_GRABBER_LEFT_GOAL = 218,
  AX_ADDR_GRABBER_RIGHT_GOAL = 222,

  AX_ADDR_PRESENT_NECK_POSITION = 226,
  
  AX_ADDR_PRESENT_GRABBER_LEFT_POSITION = 230,
  AX_ADDR_PRESENT_GRABBER_RIGHT_POSITION = 234,
  };

typedef struct ControlItemVariables{
  uint32_t model_inform;

  uint32_t dev_time_millis;
  uint32_t dev_time_micros;

  int8_t device_status;
  uint8_t heart_beat;
  bool debug_mode;
  bool is_connect_ros2_node;
  bool is_connect_motors;
  bool is_connect_manipulator;

  bool user_led[4];
  bool push_button[2];
  bool bumper[2];

  uint16_t illumination;
  uint32_t ir_sensor;
  float sornar;

  uint32_t bat_voltage_x100;
  uint32_t bat_percent_x100;

  uint8_t buzzer_sound;

  bool imu_recalibration;
  float angular_vel[3];
  float linear_acc[3];
  float magnetic[3];
  float orientation[4];

  int32_t present_position[MotorLocation::MOTOR_NUM_MAX];
  int32_t present_velocity[MotorLocation::MOTOR_NUM_MAX];
  int32_t present_current[MotorLocation::MOTOR_NUM_MAX];

  bool motor_torque_enable_state;
  int32_t cmd_vel_linear[3];
  int32_t cmd_vel_angular[3];
  uint32_t profile_acceleration[MotorLocation::MOTOR_NUM_MAX];

  // AX motors (protocol 1.0)
  bool ax_torque_enable_state;
  int32_t ax_neck_goal;
  int32_t ax_grab_left_goal;
  int32_t ax_grab_right_goal;
  uint32_t ax_present_neck_position;
  uint32_t ax_present_grab_left_position;
  uint32_t ax_present_grab_right_position;

}ControlItemVariables;

static ControlItemVariables control_items;

// Pending AX (Protocol 1.0) commands to be flushed during Proto-1 window
struct AxPending {
  bool torque_pending = false;
  bool torque_on = false;
  bool neck_pending = false;
  uint16_t neck_goal = 0;
  bool left_pending = false;
  uint16_t left_goal = 0;
  bool right_pending = false;
  uint16_t right_goal = 0;
};
static AxPending ax_pending;


/*******************************************************************************
* Definition for TurtleBot3Core 'begin()' function
*******************************************************************************/
void TurtleBot3Core::begin(const char* model_name)
{
  uint16_t model_motor_rpm;

  if(strcmp(model_name, "Mecanumbot") == 0){
    p_tb3_model_info = &mecanumbot_info;
    model_motor_rpm = 77;
  }else{
    p_tb3_model_info = &mecanumbot_info;
    model_motor_rpm = 77;
  }

  max_linear_velocity = p_tb3_model_info->wheel_radius*2*PI*model_motor_rpm/60; // TODO
  min_linear_velocity = -max_linear_velocity;
  max_angular_velocity = max_linear_velocity/p_tb3_model_info->turning_radius;
  min_angular_velocity = -max_angular_velocity;

  bool ret; (void)ret;
  // DEBUG_SERIAL_BEGIN(57600);                    // TODO
  // DEBUG_PRINTLN(" ");
  // DEBUG_PRINTLN("Version : V221004R1");
  // DEBUG_PRINTLN("Begin Start...");

  // Setting for Dynamixel motors
  ret = motor_driver.init();
  //DEBUG_PRINTLN(ret==true?"Motor driver setup completed.":"Motor driver setup failed.");
  // Setting for IMU
  ret = sensors.init();
  //DEBUG_PRINTLN(ret==true?"Sensors setup completed.":"Sensors setup failed.");
  // Init diagnosis
  ret = diagnosis.init();
  //DEBUG_PRINTLN(ret==true?"Diagnosis setup completed.":"Diagnosis setup failed.");
  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  ret = controllers.init(max_linear_velocity, max_angular_velocity);
  //DEBUG_PRINTLN(ret==true?"RC100 Controller setup completed.":"RC100 Controller setup failed.");

  //DEBUG_PRINT("Dynamixel2Arduino Item Max : ");
  //DEBUG_PRINTLN(CONTROL_ITEM_MAX);

  control_items.debug_mode = false;
  control_items.is_connect_ros2_node = false;
  control_items.is_connect_manipulator = false;  

  // Port begin
  dxl_slave.begin();
  // Init DXL Slave function
  dxl_slave.setPortProtocolVersion(PROTOCOL_VERSION_DXL_SLAVE);
  dxl_slave.setFirmwareVersion(FIRMWARE_VER);
  dxl_slave.setID(ID_DXL_SLAVE);

  /* Add control items for Slave */
  // Items for model information of device
  control_items.model_inform = p_tb3_model_info->model_info;
  dxl_slave.addControlItem(ADDR_MODEL_INFORM, control_items.model_inform);
  // Items for Timer of device
  dxl_slave.addControlItem(ADDR_MILLIS, control_items.dev_time_millis);

  // Items to debug mode
  dxl_slave.addControlItem(ADDR_DEBUG_MODE, control_items.debug_mode);
  // Items to connect ros2
  dxl_slave.addControlItem(ADDR_CONNECT_ROS2, control_items.is_connect_ros2_node);
  // Items to connect manipulator
  dxl_slave.addControlItem(ADDR_CONNECT_MANIP, control_items.is_connect_manipulator);

  // Items to inform device status
  dxl_slave.addControlItem(ADDR_DEVICE_STATUS, control_items.device_status);
  // Items to check connection state with node
  dxl_slave.addControlItem(ADDR_HEARTBEAT, control_items.heart_beat);
  // Items for GPIO
  dxl_slave.addControlItem(ADDR_USER_LED_1, control_items.user_led[0]);
  dxl_slave.addControlItem(ADDR_USER_LED_2, control_items.user_led[1]);
  dxl_slave.addControlItem(ADDR_USER_LED_3, control_items.user_led[2]);
  dxl_slave.addControlItem(ADDR_USER_LED_4, control_items.user_led[3]);
  dxl_slave.addControlItem(ADDR_BUTTON_1, control_items.push_button[0]);
  dxl_slave.addControlItem(ADDR_BUTTON_2, control_items.push_button[1]);
  dxl_slave.addControlItem(ADDR_BUMPER_1, control_items.bumper[0]);
  dxl_slave.addControlItem(ADDR_BUMPER_2, control_items.bumper[1]);
  // Items for Analog sensors
  dxl_slave.addControlItem(ADDR_ILLUMINATION, control_items.illumination);
  dxl_slave.addControlItem(ADDR_IR, control_items.ir_sensor);
  dxl_slave.addControlItem(ADDR_SORNA, control_items.sornar);
  // Items for Battery
  dxl_slave.addControlItem(ADDR_BATTERY_VOLTAGE, control_items.bat_voltage_x100);
  dxl_slave.addControlItem(ADDR_BATTERY_PERCENT, control_items.bat_percent_x100);
  // Items for Buzzer
  dxl_slave.addControlItem(ADDR_SOUND, control_items.buzzer_sound);
  // Items for IMU
  dxl_slave.addControlItem(ADDR_IMU_RECALIBRATION, control_items.imu_recalibration);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_X, control_items.angular_vel[0]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Y, control_items.angular_vel[1]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Z, control_items.angular_vel[2]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_X, control_items.linear_acc[0]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Y, control_items.linear_acc[1]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Z, control_items.linear_acc[2]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_X, control_items.magnetic[0]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_Y, control_items.magnetic[1]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_Z, control_items.magnetic[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_W, control_items.orientation[0]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_X, control_items.orientation[1]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Y, control_items.orientation[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Z, control_items.orientation[3]);
  // Items to check status of motors
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_FL, control_items.present_position[MotorLocation::FRONT_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_FR, control_items.present_position[MotorLocation::FRONT_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_BL, control_items.present_position[MotorLocation::BACK_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_BR, control_items.present_position[MotorLocation::BACK_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_FL, control_items.present_velocity[MotorLocation::FRONT_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_FR, control_items.present_velocity[MotorLocation::FRONT_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_BL, control_items.present_velocity[MotorLocation::BACK_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_BR, control_items.present_velocity[MotorLocation::BACK_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_FL, control_items.present_current[MotorLocation::FRONT_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_FR, control_items.present_current[MotorLocation::FRONT_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_BL, control_items.present_current[MotorLocation::BACK_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_BR, control_items.present_current[MotorLocation::BACK_RIGHT]);

  // Items to control motors
  dxl_slave.addControlItem(ADDR_MOTOR_CONNECT, control_items.is_connect_motors);
  dxl_slave.addControlItem(ADDR_MOTOR_TORQUE, control_items.motor_torque_enable_state);
  dxl_slave.addControlItem(ADDR_CMD_VEL_LINEAR_X, control_items.cmd_vel_linear[0]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_LINEAR_Y, control_items.cmd_vel_linear[1]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_LINEAR_Z, control_items.cmd_vel_linear[2]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_ANGULAR_X, control_items.cmd_vel_angular[0]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_ANGULAR_Y, control_items.cmd_vel_angular[1]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_ANGULAR_Z, control_items.cmd_vel_angular[2]);  
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_FL, control_items.profile_acceleration[MotorLocation::FRONT_LEFT]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_FR, control_items.profile_acceleration[MotorLocation::FRONT_RIGHT]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_BL, control_items.profile_acceleration[MotorLocation::BACK_LEFT]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_BR, control_items.profile_acceleration[MotorLocation::BACK_RIGHT]);

  // AX motors control items (user-defined)
  dxl_slave.addControlItem(AX_ADDR_TORQUE, control_items.ax_torque_enable_state);
  dxl_slave.addControlItem(AX_ADDR_NECK_GOAL, control_items.ax_neck_goal);
  dxl_slave.addControlItem(AX_ADDR_GRABBER_LEFT_GOAL, control_items.ax_grab_left_goal);
  dxl_slave.addControlItem(AX_ADDR_GRABBER_RIGHT_GOAL, control_items.ax_grab_right_goal);
  dxl_slave.addControlItem(AX_ADDR_PRESENT_NECK_POSITION, control_items.ax_present_neck_position);
  dxl_slave.addControlItem(AX_ADDR_PRESENT_GRABBER_LEFT_POSITION, control_items.ax_present_grab_left_position);
  dxl_slave.addControlItem(AX_ADDR_PRESENT_GRABBER_RIGHT_POSITION, control_items.ax_present_grab_right_position);

  // Set user callback function for processing write command from master.
  dxl_slave.setWriteCallbackFunc(dxl_slave_write_callback_func);

  // Check connection state with motors.
  if(motor_driver.is_connected() == true){
    motor_driver.set_torque(true);
    control_items.device_status = STATUS_RUNNING;
    set_connection_state_with_motors(true);
    //DEBUG_PRINTLN("Wheel motors are connected");
  }else{
    control_items.device_status = STATUS_NOT_CONNECTED_MOTORS;
    set_connection_state_with_motors(false);
    //DEBUG_PRINTLN("Can't communicate with the motor!");
    //DEBUG_PRINTLN("  Please check the connection to the motor and the power supply.");
    //DEBUG_PRINTLN();
  } 
  control_items.is_connect_motors = get_connection_state_with_motors();  

  // Init AX motors (Neck/Grabbers)
  ax_driver.init();
  if (ax_driver.is_connected()) {
    set_connection_state_with_joints(true);
    control_items.is_connect_manipulator = true;
    ax_driver.setTorqueInit(true);
  }

  // Init IMU 
  sensors.initIMU();
  sensors.calibrationGyro();

  //To indicate that the initialization is complete.
  sensors.makeMelody(1); 

  if (get_connection_state_with_motors() == true) {
    const float boot_lin = 0.1f; // small linear speed (m/s)

    // forward
    motor_driver.control_motors(p_tb3_model_info->wheel_separation_x,
                                p_tb3_model_info->wheel_separation_y,
                                boot_lin, 0.0f, 0.0f);
    delay(200);

    // reverse
    motor_driver.control_motors(p_tb3_model_info->wheel_separation_x,
                                p_tb3_model_info->wheel_separation_y,
                                -boot_lin, 0.0f, 0.0f);
    delay(200);

    // stop
    motor_driver.control_motors(p_tb3_model_info->wheel_separation_x,
                                p_tb3_model_info->wheel_separation_y,
                                0.0f, 0.0f, 0.0f);
  }
  else {
    //DEBUG_PRINTLN("Get connection state with motors returned with FALSE");
  }

  // Brief AX motion at boot if connected (IDs: Neck=7, Left=6, Right=5)
  if (get_connection_state_with_joints()) {
    ax_driver.setProtoV1_(); // switch to Protocol 1.0
    const uint16_t center = 800; // ~0 degrees
    const uint16_t d90deg = 90; // ~90 degrees
    // Neck nod
    ax_driver.writeGoal(7, center);
    //ax_driver.writeGoal(7, center);
    // Grabbers open/close (tune these as needed)
    const uint16_t openPos = 350;
    const uint16_t closePos = 700;
    const uint16_t midPos = 512;
    ax_driver.writeGoal(6, midPos);
    ax_driver.writeGoal(5, midPos);
    //delay(600);

    //ax_driver.setTorque(false); // disable torque after motion
    ax_driver.restoreProtoV2_(); // switch back to Protocol 2.0
  }

  //DEBUG_PRINTLN("Begin End...");
}

/*******************************************************************************
* Definition for TurtleBot3Core 'run()' function
*******************************************************************************/
bool switcher_ax_motors = false;
void TurtleBot3Core::run()
{
  static uint32_t pre_time_to_control_motor;

  // Check connection state with ROS2 node
  update_connection_state_with_ros2_node();

  /* For diagnosis */
  // Show LED status
  diagnosis.showLedStatus(get_connection_state_with_ros2_node());
  // Update Voltage
  diagnosis.updateVoltageCheck(true);
  // Check push button pressed for simple test drive
  test_motors_with_buttons(diagnosis.getButtonPress(3000));

  /* For sensing and run buzzer */
  // Update the IMU unit
  sensors.updateIMU();
  // Update sonar data
  // TODO: sensors.updateSonar(t);
  // Run buzzer if there is still melody to play.
  sensors.onMelody();

  /* For getting command from rc100 */
  // Receive data from RC100 
  controllers.getRCdata(goal_velocity_from_rc100, goal_state_from_rc100);

  /* For processing DYNAMIXEL slave function */
  // Update control table of OpenCR to communicate with ROS2 node
  update_imu(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_times(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_gpios(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);

  // Alternate 20 ms protocol windows: Proto2 (XM/wheels) <-> Proto1 (AX/joints)
  if (switcher_ax_motors == false) {
    // Protocol 2.0 window: wheel XM motors
    process_wheel_writes(INTERVAL_MS_TO_CONTROL_MOTOR);
    update_motor_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
    switcher_ax_motors = true;
  } else {
    // Protocol 1.0 window: AX-12 (neck/grabbers)
    //setProtoV1_();
    ax_driver.setProtoV1_();
    process_ax_writes(INTERVAL_MS_TO_CONTROL_MOTOR);
    update_ax_motor_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
    ax_driver.restoreProtoV2_();
    switcher_ax_motors = false;
  }
  

  // Update AX (neck/grabbers) status into control table at a lower rate

  update_battery_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_analog_sensors(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);

  // Packet processing with ROS2 Node.
  dxl_slave.processPacket();

  // Wheel writes are handled in the Protocol 2.0 window by process_wheel_writes()
}


/*******************************************************************************
* Function definition for updating velocity values 
* to be used for control of DYNAMIXEL(motors).
*******************************************************************************/
void update_goal_velocity_from_3values(void)
{
  goal_velocity[VelocityType::LINEAR_X]  = goal_velocity_from_button[VelocityType::LINEAR_X]  + goal_velocity_from_cmd[VelocityType::LINEAR_X]  + goal_velocity_from_rc100[VelocityType::LINEAR_X];
  goal_velocity[VelocityType::LINEAR_Y]  = goal_velocity_from_button[VelocityType::LINEAR_Y]  + goal_velocity_from_cmd[VelocityType::LINEAR_Y]  + goal_velocity_from_rc100[VelocityType::LINEAR_Y];
  goal_velocity[VelocityType::ANGULAR] = goal_velocity_from_button[VelocityType::ANGULAR] + goal_velocity_from_cmd[VelocityType::ANGULAR] + goal_velocity_from_rc100[VelocityType::ANGULAR];

  sensors.setLedPattern(goal_velocity[VelocityType::LINEAR_X], goal_velocity[VelocityType::ANGULAR]);
}


/*******************************************************************************
* Function definition for updating control items in TB3.
*******************************************************************************/
float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_times(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    control_items.dev_time_millis = millis();
    control_items.dev_time_micros = micros();
  } 
}

void update_gpios(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    control_items.user_led[0] = digitalRead(BDPIN_GPIO_4);
    control_items.user_led[1] = digitalRead(BDPIN_GPIO_6);
    control_items.user_led[2] = digitalRead(BDPIN_GPIO_8);
    control_items.user_led[3] = digitalRead(BDPIN_GPIO_10);

    control_items.push_button[0] = digitalRead(BDPIN_PUSH_SW_1);
    control_items.push_button[1] = digitalRead(BDPIN_PUSH_SW_2);

    control_items.bumper[0] = sensors.getBumper1State();
    control_items.bumper[1] = sensors.getBumper2State();
  }  
}

void update_battery_status(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float bat_voltage, bat_percent;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    bat_voltage = sensors.checkVoltage();
    control_items.bat_voltage_x100 = (uint32_t)(bat_voltage*100);

    if(bat_voltage >= 3.5*3){
      bat_percent = map_float(bat_voltage, 3.5*3, 4.1*3, 0.0, 100.0);
      control_items.bat_percent_x100 = (uint32_t)(bat_percent*100);
    }
  }
}

void update_analog_sensors(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    control_items.illumination = (uint16_t)sensors.getIlluminationData();
    control_items.ir_sensor = (uint32_t)sensors.getIRsensorData();
    control_items.sornar = (float)sensors.getSonarData();
  }
}

void update_imu(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float* p_imu_data;

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();

    p_imu_data = sensors.getImuAngularVelocity();
    memcpy(control_items.angular_vel, p_imu_data, sizeof(control_items.angular_vel));

    p_imu_data = sensors.getImuLinearAcc();
    memcpy(control_items.linear_acc, p_imu_data, sizeof(control_items.linear_acc));

    p_imu_data = sensors.getImuMagnetic();
    memcpy(control_items.magnetic, p_imu_data, sizeof(control_items.magnetic));

    p_imu_data = sensors.getOrientation();
    memcpy(control_items.orientation, p_imu_data, sizeof(control_items.orientation));
  }  
}

void update_motor_status(uint32_t interval_ms)
{
  static uint32_t pre_time;
  int16_t current_fl, current_fr, current_bl, current_br;
  static uint32_t pre_time_torque = 0; // throttle torque polling

  if(millis() - pre_time >= interval_ms){
    pre_time = millis();


    uint32_t pre_time_dxl;

    pre_time_dxl = millis();
    if(get_connection_state_with_motors() == true){
      motor_driver.read_present_position(control_items.present_position[MotorLocation::FRONT_LEFT], control_items.present_position[MotorLocation::FRONT_RIGHT], control_items.present_position[MotorLocation::BACK_LEFT], control_items.present_position[MotorLocation::BACK_RIGHT]);
      motor_driver.read_present_velocity(control_items.present_velocity[MotorLocation::FRONT_LEFT], control_items.present_velocity[MotorLocation::FRONT_RIGHT], control_items.present_velocity[MotorLocation::BACK_LEFT], control_items.present_velocity[MotorLocation::BACK_RIGHT]);
      if(motor_driver.read_present_current(current_fl, current_fr, current_br, current_bl) == true){
        control_items.present_current[MotorLocation::FRONT_LEFT]  = current_fl;
        control_items.present_current[MotorLocation::FRONT_RIGHT] = current_fr;
        control_items.present_current[MotorLocation::BACK_LEFT]   = current_bl;
        control_items.present_current[MotorLocation::BACK_RIGHT]  = current_br;
      }
      // Read torque state at most once per second to avoid 4 blocking reads every cycle
      if (millis() - pre_time_torque >= 1000) {
        pre_time_torque = millis();
        control_items.motor_torque_enable_state = motor_driver.get_torque();
      }
    }
  }  
}

void update_ax_motor_status(uint32_t interval_ms)
{
    static uint32_t pre_time = 0;
    if (millis() - pre_time >= interval_ms) {
      pre_time = millis();
      uint16_t p;
      if (get_connection_state_with_joints() == true) {
        if (ax_driver.readPresent(7, p)) control_items.ax_present_neck_position = p;  
        if (ax_driver.readPresent(6, p)) control_items.ax_present_grab_left_position = p;  
        if (ax_driver.readPresent(5, p)) control_items.ax_present_grab_right_position = p;  
      }
    }
  }


/*******************************************************************************
* Callback function definition to be used in communication with the ROS2 node.
*******************************************************************************/
static void dxl_slave_write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)arg;

  switch(item_addr)
  {
    case ADDR_MODEL_INFORM:
      control_items.model_inform = p_tb3_model_info->model_info;
      dxl_err_code = DXL_ERR_ACCESS;
      break;

    // case ADDR_DEBUG_MODE:
    //   if (control_items.debug_mode == true)
    //     //DEBUG_PRINTLN("Debug Mode : Enabled");
    //   else
    //     //DEBUG_PRINTLN("Debug Mode : Disabled");
    //   break;

    case ADDR_SOUND:
      sensors.makeMelody(control_items.buzzer_sound);
      break;

    case ADDR_IMU_RECALIBRATION:
      if(control_items.imu_recalibration == true){
        sensors.calibrationGyro();
        control_items.imu_recalibration = false;
      }
      break;

    case ADDR_MOTOR_TORQUE:
      if(get_connection_state_with_motors() == true)
        motor_driver.set_torque(control_items.motor_torque_enable_state);
      break;

    case ADDR_CMD_VEL_LINEAR_X:
      goal_velocity_from_cmd[VelocityType::LINEAR_X] = constrain((float)(control_items.cmd_vel_linear[0]*0.01f), min_linear_velocity, max_linear_velocity);
      break;

    case ADDR_CMD_VEL_LINEAR_Y:
      goal_velocity_from_cmd[VelocityType::LINEAR_Y] = constrain((float)(control_items.cmd_vel_linear[1]*0.01f), min_linear_velocity, max_linear_velocity);
      break;

    case ADDR_CMD_VEL_ANGULAR_Z:
      goal_velocity_from_cmd[VelocityType::ANGULAR] = constrain((float)(control_items.cmd_vel_angular[2]*0.01f), min_angular_velocity, max_angular_velocity);
      break;            
    
    // case ADDR_PROFILE_ACC_BL:
    // case ADDR_PROFILE_ACC_BR:
    // case ADDR_PROFILE_ACC_FL:
    // case ADDR_PROFILE_ACC_FR:
    
    //   if(get_connection_state_with_motors() == true)
    //     motor_driver.write_profile_acceleration(control_items.profile_acceleration[MotorLocation::FRONT_RIGHT], control_items.profile_acceleration[MotorLocation::FRONT_LEFT], control_items.profile_acceleration[MotorLocation::BACK_RIGHT], control_items.profile_acceleration[MotorLocation::BACK_LEFT]);
    //   break;        

    // AX motors control via slave table
    // 7 - Neck, 6 - Left Grabber, 5 - Right Grabber
    case AX_ADDR_TORQUE:
      // Buffer torque change; apply in Protocol 1 window
      ax_pending.torque_on = control_items.ax_torque_enable_state;
      ax_pending.torque_pending = true;
      break;
    case AX_ADDR_NECK_GOAL: {
      // Constrain and buffer; apply in Protocol 1 window
      int16_t v = constrain(control_items.ax_neck_goal, 200, 860);
      control_items.ax_neck_goal = v;
      ax_pending.neck_goal = (uint16_t)v;
      ax_pending.neck_pending = true;
      break;
    }
    case AX_ADDR_GRABBER_LEFT_GOAL: {
      int16_t v = constrain(control_items.ax_grab_left_goal, 160, 854);
      control_items.ax_grab_left_goal = v;
      ax_pending.left_goal = (uint16_t)v;
      ax_pending.left_pending = true;
      break;
    }
    case AX_ADDR_GRABBER_RIGHT_GOAL: {
      int16_t v = constrain(control_items.ax_grab_right_goal, 160, 854);
      control_items.ax_grab_right_goal = v;
      ax_pending.right_goal = (uint16_t)v;
      ax_pending.right_pending = true;
      break;
    }
  }
      
}

// Apply buffered AX writes within the Protocol 1.0 window at most once per interval
static void process_ax_writes(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  if (millis() - pre_time < interval_ms) return;
  pre_time = millis();

  if (!get_connection_state_with_joints()) return;

  // Apply pending torque first to ensure safe motion
  if (ax_pending.torque_pending) {
    ax_driver.setTorque(ax_pending.torque_on);
    ax_pending.torque_pending = false;
  }
  // Apply goals if pending
  if (ax_pending.neck_pending) {
    ax_driver.writeGoal(7, ax_pending.neck_goal);
    ax_pending.neck_pending = false;
  }
  if (ax_pending.left_pending) {
    ax_driver.writeGoal(6, ax_pending.left_goal);
    ax_pending.left_pending = false;
  }
  if (ax_pending.right_pending) {
    ax_driver.writeGoal(5, ax_pending.right_goal);
    ax_pending.right_pending = false;
  }
}

// Handle wheel control writes within the Protocol 2.0 window at most once per interval
static void process_wheel_writes(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  if (millis() - pre_time < interval_ms) return;
  pre_time = millis();

  if(get_connection_state_with_ros2_node() == false){
    memset(goal_velocity_from_cmd, 0, sizeof(goal_velocity_from_cmd));
  }
  update_goal_velocity_from_3values();
  if(get_connection_state_with_motors() == true){
    motor_driver.control_motors(p_tb3_model_info->wheel_separation_x, p_tb3_model_info->wheel_separation_y, goal_velocity[VelocityType::LINEAR_X], goal_velocity[VelocityType::LINEAR_Y], goal_velocity[VelocityType::ANGULAR]);
  }
}


/*******************************************************************************
* Function definition to check the connection status with the ROS2 node.
*******************************************************************************/
static bool connection_state_with_ros2_node = false;

static bool get_connection_state_with_ros2_node()
{
  return connection_state_with_ros2_node;
}

static void set_connection_state_with_ros2_node(bool is_connected)
{
  connection_state_with_ros2_node = is_connected;
}

void update_connection_state_with_ros2_node()
{
  static uint32_t pre_time;
  static uint8_t pre_data;
  static bool pre_state;

  //To wait for IMU Calibration
  if(pre_state != get_connection_state_with_ros2_node()){
    pre_state = get_connection_state_with_ros2_node();
    pre_time = millis();
    return;
  }

  if(pre_data != control_items.heart_beat || control_items.debug_mode == true){
    pre_time = millis();
    pre_data = control_items.heart_beat;
    set_connection_state_with_ros2_node(true);
  }else{
    if(millis()-pre_time >= HEARTBEAT_TIMEOUT_MS){
      pre_time = millis();
      set_connection_state_with_ros2_node(false);
    }
  }

  control_items.is_connect_ros2_node = get_connection_state_with_ros2_node();
}


/*******************************************************************************
* Function definition to check the connection with the motor.
*******************************************************************************/
static bool is_connected_motors = false;

static bool get_connection_state_with_motors()
{
  return is_connected_motors;
}

static void set_connection_state_with_motors(bool is_connected)
{
  is_connected_motors = is_connected;
}

/*******************************************************************************
* Function definition to check the connection with the motor.
*******************************************************************************/
static bool is_connected_joints = false;

static bool get_connection_state_with_joints()
{
  return is_connected_joints;
}

static void set_connection_state_with_joints(bool is_connected)
{
  is_connected_joints = is_connected;
}

/*******************************************************************************
* Function definition to test motors using the built-in buttons of OpenCR.
*******************************************************************************/
const float TICK2RAD = 0.001533981; // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
const float TEST_DISTANCE = 0.300; // meter
const float TEST_RADIAN = 3.14; // 180 degree

void test_motors_with_buttons(uint8_t buttons)
{
  static bool move[VelocityType::TYPE_NUM_MAX] = {false, false, false};
  static int32_t saved_tick[MotorLocation::MOTOR_NUM_MAX] = {0, 0, 0, 0};
  static double diff_encoder = 0.0;

  int32_t current_tick[2] = {0, 0};

  if(get_connection_state_with_motors() == true){
    motor_driver.read_present_position(current_tick[MotorLocation::FRONT_LEFT], current_tick[MotorLocation::FRONT_RIGHT], current_tick[MotorLocation::BACK_RIGHT], current_tick[MotorLocation::BACK_LEFT]);
  }

  if (buttons & (1<<0))  
  {
    move[VelocityType::LINEAR_X] = true;
    saved_tick[MotorLocation::FRONT_RIGHT] = current_tick[MotorLocation::FRONT_RIGHT];

    diff_encoder = TEST_DISTANCE / (0.207 / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
  }
  else if (buttons & (1<<1))
  {
    move[VelocityType::ANGULAR] = true;
    saved_tick[MotorLocation::FRONT_RIGHT] = current_tick[MotorLocation::FRONT_RIGHT];

    diff_encoder = (TEST_RADIAN * p_tb3_model_info->turning_radius) / (0.207 / 4096);
  }

  if (move[VelocityType::LINEAR_X])
  {
    if (abs(saved_tick[MotorLocation::FRONT_RIGHT] - current_tick[MotorLocation::FRONT_RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[VelocityType::LINEAR_X]  = 0.05;
    }
    else
    {
      goal_velocity_from_button[VelocityType::LINEAR_X]  = 0.0;
      move[VelocityType::LINEAR_X] = false;
    }
  }
  else if (move[VelocityType::ANGULAR])
  {
    if (abs(saved_tick[MotorLocation::FRONT_RIGHT] - current_tick[MotorLocation::FRONT_RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[VelocityType::ANGULAR]= -0.7;
    }
    else
    {
      goal_velocity_from_button[VelocityType::ANGULAR]  = 0.0;
      move[VelocityType::ANGULAR] = false;
    }
  }
}