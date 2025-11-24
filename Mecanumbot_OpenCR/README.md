# This file contains the documentaton of the low level Mecanumbot microcontroller, Mecanumbot - OpenCR

## mecanumbot.ino

```
void setup() {
    Serial.begin(57600);
    Serial.println("Hello, Mecanumbot A New Age!");
    MecanumbotCore::begin();
}
```
* Starts the serial communication, this will be the primary (only) communication chanel between the Arduino and the OpenCR
* Calls the MecanumbotCore::begin() from mecanumbot.h
```
void loop() {
    MecanumbotCore::run(); 
}
```
* Calls the MecanumbotCore::run from mecanumbot.h after the setup is completed (the begin function) until termination

## mecanumbot.h

```
#define DEVICENAME               "OpenCR_DXL_Port"  // For OpenCR
#define BAUDRATE                 1000000
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
```
* Manually set constants specifically for the mecanum bot
* Wheel IDs start from the back left. 
* The working parametres are hand measured

```
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
```
* The parametres of the AX motors. (Grabbers and Neck)
* The constants come from the documentation of the motors (https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
```
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
```
* Parametres for the XM motors. (Wheels)
* The constants come from the original documentation (https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/)
```
namespace MecanumbotCore {
  void begin();
  void run();
}
```
Defining the mecanumbot namespace "MecanumbotCore"

## mecanumbot.cpp

```
dynamixel::PortHandler *portHandler       = nullptr;
dynamixel::PacketHandler *packetHandlerAX = nullptr;
dynamixel::PacketHandler *packetHandlerXM = nullptr;
dynamixel::GroupSyncWrite *groupSyncWriteXM = nullptr;
dynamixel::GroupSyncWrite *groupSyncWriteAX = nullptr;
```
* Creating the port handlers and packet handlers
* Important to note that this robot uses two protocols (protocol 1.0 for AX and protocol 2.0 for XM), therfore we need two packet handlers and group sync writes, one for each. 
* The port handler needs to stay singular and the sole communicator on the USB connection to avoid packet smashing
```
void writeByte(dynamixel::PacketHandler* handler, int ID, int ADDRESS, uint16_t DATA) { 
  uint8_t dxl_error;

  int dxl_comm_result = handler->write2ByteTxRx(portHandler, ID, ADDRESS, DATA, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print("COMM Error: ");
    Serial.println(handler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    Serial.print("Error Code: ");
    Serial.println(handler->getRxPacketError(dxl_error));
  }
}
```
* The packet handler tries to write a packet on the motor with the specified id to the given address. 
* If we want to enable torque on the back left motor we need the XM packet handler to write "1" to address "64" of motor "1" (i.e. writeByte(packetHandlerXM, ID, XM_ADDR_TORQUE_ENABLE, TORQUE_ENABLE))
```
void set_WheelVelocities(int32_t vBL, int32_t vBR, int32_t vFL, int32_t vFR) {
  uint8_t param_goal_velocity_BL[4] = {
    DXL_LOBYTE(DXL_LOWORD(vBL)),
    DXL_HIBYTE(DXL_LOWORD(vBL)),
    DXL_LOBYTE(DXL_HIWORD(vBL)),
    DXL_HIBYTE(DXL_HIWORD(vBL))
  };

  ...

  groupSyncWriteXM->addParam(ID_WHEEL_FR, param_goal_velocity_FR);

  groupSyncWriteXM->txPacket();
  groupSyncWriteXM->clearParam();
}
```
* Sets the wheel velocities in a synchronous way
* Dynamixel protocol packets send multi-byte params as little-endians. --> split vBL (velocity back left) into 4 bytes to send over the bus
* Add the parametres to the sync write so that all the wheels receive the information at the same time to avoid latency-induced turning
```
void set_AXPositions(int32_t pN, int32_t pGL, int32_t pGR) {...}
```
* Same as above but for the AX motors
```
void initializeXMMotors(int ID) {
    writeByte(packetHandlerXM, ID, XM_ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    writeByte(packetHandlerXM, ID, XM_ADDR_OPERATING_MODE, XM_VELOCITY_MODE);
    writeByte(packetHandlerXM, ID, XM_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
}
```
* Initialises the XM motors by disabling torque enabling velocity mode, then enabling torque. (Refer to the XM docs mentioned above)
```
void initializeMotors() {
    initializeXMMotors(ID_WHEEL_BL);
    ...
    writeByte(packetHandlerAX, ID_GRABBER_R, AX_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    ...
}
```
* Initialise both XM and AX motors, by enabling torque. (The latter part is kind of redundant as it is done by default, but its better to be sure)

```
void initDXLConnection() {
    if (portHandler->openPort()) {
        Serial.println("Opened Dynamixel port successfully.");
    } else {
        Serial.println("Failed to open Dynamixel port!");
        return;
    }

    if (portHandler->setBaudRate(BAUDRATE)) {
        Serial.println("Set Dynamixel baudrate successfully.");
    } else {
        Serial.println("Failed to set Dynamixel baudrate!");
        return;
    }
}
```
* Try to initialise the Dynamixel connection; open the "OpenCR_DXL_Port" for communicating with the motors
* if the port can be opened set the baudrate. (! Important to match the baudrate here with the baudrate of the motors. (Can be checked using Dynamixel wizzard: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/))

### Commnication aiding structs
The communication happens over a wired, USB, connection. The robot receives the move commands in a septuple and sends back its sensory state.

```
struct __attribute__((packed)) SensorData {
    ...
};

SensorData sensorData;
```
* Defines the packet that will be travelling on the USB. This contains all the sensory information. 
* packed parametre is crucial, for encoding and decoding integrity

```
struct __attribute__((packed)) ControlData {
  //goal velocities for wheels  
  int16_t vel_BL = 0;
  int16_t vel_BR = 0;
  int16_t vel_FL = 0;
  int16_t vel_FR = 0;
  //goal positions for neck and grabbers
  int16_t pos_N = 0;
  int16_t pos_GL = 0;
  int16_t pos_GR = 0;
};

ControlData controlData;
```
* A septuple containing goal velocities and goal positions. This is the primary way of giving movement commands to the robot. 

```
static uint8_t crc8_ccitt(const uint8_t *data, size_t len)
{
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x80)
        crc = (uint8_t)((crc << 1) ^ 0x07);
      else
        crc <<= 1;
    }
  }
  return crc;
}
```
* Compute an 8-bit crc. (cyclic redundancy check)
* Used as a checksum to combat data corruption. (Is also needed on the other side of the communication. See: [github.com/fegyobeno](https://github.com/fegyobeno/mecanumbot_python/blob/main/new_age_python.py))

```
struct __attribute__((packed)) SensorPacket {
  uint16_t magic;
  uint8_t seq;
  SensorData payload;
  uint8_t crc;
};
```
* The final format of the package being sent on the com.
* Magic byte in the front (where the message begins)
* Content of the message
* Checksum at the end

### MecanumbotCore::begin()
```
portHandler       = dynamixel::PortHandler::getPortHandler(DEVICENAME);
packetHandlerAX   = dynamixel::PacketHandler::getPacketHandler(AX_PROTOCOL_VERSION);
packetHandlerXM   = dynamixel::PacketHandler::getPacketHandler(XM_PROTOCOL_VERSION);
groupSyncWriteXM  = new dynamixel::GroupSyncWrite(portHandler, packetHandlerXM,            XM_ADDR_GOAL_VELOCITY, XM_LEN_GOAL_VELOCITY);
groupSyncWriteAX  = new dynamixel::GroupSyncWrite(portHandler, packetHandlerAX, AX_ADDR_GOAL_POSITION, 2);
```
* Create the port and packet handler instances, alongside with the sync-writes.
```
initDXLConnection();    
initializeMotors();
sensors.init();
sensors.initIMU();
sensors.calibrationGyro();
delay(1000);
```
* Initialise the robot state (motors, sensors, IMU, Gyro) then wait a second to give everything time to load.

### MecanumbotCore::run()
```
sensors.updateIMU();
sensors.onMelody();
```
* Update IMU and play any melody in waiting
```
if (Serial.available() >= controlDataSize)
  {
    size_t bytesRead = Serial.readBytes((char*)&controlData, controlDataSize);
    if (bytesRead == controlDataSize)
    {
      // Simple processing of control packet
      set_WheelVelocities(controlData.vel_BL, controlData.vel_BR, controlData.vel_FL, controlData.vel_FR);
      set_AXPositions(controlData.pos_N, controlData.pos_GL, controlData.pos_GR);

      // Mirror control into sensorData for host visibility
      sensorData.vel_BL = controlData.vel_BL;
      sensorData.vel_BR = controlData.vel_BR;
      sensorData.vel_FL = controlData.vel_FL;
      sensorData.vel_FR = controlData.vel_FR;
      sensorData.pos_N = controlData.pos_N;
      sensorData.pos_GL = controlData.pos_GL;
      sensorData.pos_GR = controlData.pos_GR;
    }
    else
    {
      // Partial/failed read -- discard remaining bytes to resync
      while (Serial.available()) Serial.read();
    }
  }
```
* Read the serial port and check if a message can be read, that matches the length of the control data. 
* If it does, read it, write the requried data to the given motors, and update the sensory state with the values. 
```
sensorData.voltage = sensors.checkVoltage();

float* tmp;
// IMU data --> sensorData
tmp = sensors.getImuAngularVelocity(); //static float angular_vel[3];
...
tmp = sensors.getImuLinearAcc(); // static float linear_acc[3];
...

SensorPacket pkt;
```
* Collect the sensory state into "pkt"
```
pkt.magic = 0xAA55; // Magic number, very low probability of occurring randomly
pkt.seq = sensor_seq_counter++;
pkt.payload = sensorData;
pkt.crc = crc8_ccitt((const uint8_t*)&pkt, sizeof(pkt) - 1);
Serial.write((const uint8_t*)&pkt, sizeof(pkt));
```
* Create the sensor packet with the magic byte and checksum, and write it onto the serial

# Mecanumbot sensors
A skimmed version of the original turtlebot sensor state, with redundant parts removed