#include "mecanumbot.h"
#include <Wire.h>
#include <VL53L0X.h>   // "VL53L0X" by Pololu (Arduino Library Manager)

dynamixel::PortHandler *portHandler       = nullptr;
dynamixel::PacketHandler *packetHandlerAX = nullptr;
dynamixel::PacketHandler *packetHandlerXM = nullptr;
dynamixel::GroupSyncWrite *groupSyncWriteXM = nullptr;
dynamixel::GroupSyncWrite *groupSyncWriteAX = nullptr;

// --- FIX 1: DECLARE THESE GLOBALLY ---
dynamixel::GroupSyncRead *groupSyncReadVel = nullptr;
dynamixel::GroupSyncRead *groupSyncReadPos = nullptr;
// -------------------------------------

static MecanumbotSensor sensors;

// --- DMS: VL53L0X time-of-flight distance sensor on the OpenCR I2C header ---
// Replaces the HC-SR04 that was on TRIG 2 / ECHO 3 (those pins are now free).
// Wiring: VIN -> 3.3V, GND -> GND, SDA -> SDA, SCL -> SCL (Arduino header).
// Use a breakout with its own regulator; the bare chip is 2.6-3.5 V only.
//
// sensorData.dms keeps the HC-SR04 contract so the packet and the host side
// are unchanged: distance in cm, -1.0 when there is no valid reading
// (out of range, I2C error, or no fresh sample), -2.0 while the sensor is
// not initialised.
static VL53L0X tof;
static bool tof_ok = false;
static float tof_last_cm = -1.0f;
static uint32_t tof_last_sample_ms = 0;

// The chip reports ~8190/8191 mm when nothing is in range, 65535 on a timeout.
static const uint16_t TOF_INVALID_MM = 8190;
// If no new sample arrives within this window the value is stale, not live.
// Continuous back-to-back mode with the default 33 ms timing budget delivers
// one sample every ~33 ms.
static const uint32_t TOF_STALE_MS = 200;

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

void writeDword(dynamixel::PacketHandler* handler, int ID, int ADDRESS, uint32_t DATA) {
  uint8_t dxl_error;
  int dxl_comm_result = handler->write4ByteTxRx(portHandler, ID, ADDRESS, DATA, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    Serial.print("COMM Error: ");
    Serial.println(handler->getTxRxResult(dxl_comm_result));
  } else if (dxl_error != 0) {
    Serial.print("Error Code: ");
    Serial.println(handler->getRxPacketError(dxl_error));
  }
}

void set_WheelVelocities(int32_t vBL, int32_t vBR, int32_t vFL, int32_t vFR) {
  uint8_t param_goal_velocity_BL[4] = {
    DXL_LOBYTE(DXL_LOWORD(vBL)),
    DXL_HIBYTE(DXL_LOWORD(vBL)),
    DXL_LOBYTE(DXL_HIWORD(vBL)),
    DXL_HIBYTE(DXL_HIWORD(vBL))
  };

  uint8_t param_goal_velocity_BR[4] = {
    DXL_LOBYTE(DXL_LOWORD(vBR)),
    DXL_HIBYTE(DXL_LOWORD(vBR)),
    DXL_LOBYTE(DXL_HIWORD(vBR)),
    DXL_HIBYTE(DXL_HIWORD(vBR))
  };

  uint8_t param_goal_velocity_FL[4] = {
    DXL_LOBYTE(DXL_LOWORD(vFL)),
    DXL_HIBYTE(DXL_LOWORD(vFL)),
    DXL_LOBYTE(DXL_HIWORD(vFL)),
    DXL_HIBYTE(DXL_HIWORD(vFL))
  };

  uint8_t param_goal_velocity_FR[4] = {
    DXL_LOBYTE(DXL_LOWORD(vFR)),
    DXL_HIBYTE(DXL_LOWORD(vFR)),
    DXL_LOBYTE(DXL_HIWORD(vFR)),
    DXL_HIBYTE(DXL_HIWORD(vFR))
  };

  groupSyncWriteXM->addParam(ID_WHEEL_BL, param_goal_velocity_BL);
  groupSyncWriteXM->addParam(ID_WHEEL_BR, param_goal_velocity_BR);
  groupSyncWriteXM->addParam(ID_WHEEL_FL, param_goal_velocity_FL);
  groupSyncWriteXM->addParam(ID_WHEEL_FR, param_goal_velocity_FR);

  groupSyncWriteXM->txPacket();
  groupSyncWriteXM->clearParam();
}

void set_AXPositions(int32_t pN, int32_t pGL, int32_t pGR) {	
  uint8_t param_goal_position_GL[4] = {
    DXL_LOBYTE(DXL_LOWORD(pGL)),
    DXL_HIBYTE(DXL_LOWORD(pGL)),
    DXL_LOBYTE(DXL_HIWORD(pGL)),
    DXL_HIBYTE(DXL_HIWORD(pGL))
  };

  uint8_t param_goal_position_GR[4] = {
    DXL_LOBYTE(DXL_LOWORD(pGR)),
    DXL_HIBYTE(DXL_LOWORD(pGR)),
    DXL_LOBYTE(DXL_HIWORD(pGR)),
    DXL_HIBYTE(DXL_HIWORD(pGR))
  };

  uint8_t param_goal_position_N[4] = {
    DXL_LOBYTE(DXL_LOWORD(pN)),
    DXL_HIBYTE(DXL_LOWORD(pN)),
    DXL_LOBYTE(DXL_HIWORD(pN)),
    DXL_HIBYTE(DXL_HIWORD(pN))
  };

  groupSyncWriteAX->addParam(ID_GRABBER_L, param_goal_position_GL);
  groupSyncWriteAX->addParam(ID_GRABBER_R, param_goal_position_GR);
  groupSyncWriteAX->addParam(ID_NECK, param_goal_position_N);
  groupSyncWriteAX->txPacket();
  groupSyncWriteAX->clearParam();
}

void initializeXMMotors(int ID) {
    writeByte(packetHandlerXM, ID, XM_ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    writeByte(packetHandlerXM, ID, XM_ADDR_OPERATING_MODE, XM_VELOCITY_MODE);
    // Ramp instead of step. This is RAM and defaults back to 0 on every power
    // cycle, so it belongs here rather than in a one-off Wizard session.
    writeDword(packetHandlerXM, ID, XM_ADDR_PROFILE_ACCELERATION, XM_PROFILE_ACCELERATION_VALUE);
    writeByte(packetHandlerXM, ID, XM_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
}

void initializeMotors() {
  
    initializeXMMotors(ID_WHEEL_BL);
    initializeXMMotors(ID_WHEEL_BR);
    initializeXMMotors(ID_WHEEL_FL);
    initializeXMMotors(ID_WHEEL_FR);
    writeByte(packetHandlerAX, ID_GRABBER_R, AX_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    writeByte(packetHandlerAX, ID_GRABBER_L, AX_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    writeByte(packetHandlerAX, ID_NECK, AX_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

} 

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

// dms value while the sensor is not initialised (not found / init failed),
// so it can be told apart from -1 "no target in range" in the telemetry.
static const float TOF_NOT_READY = -2.0f;
static const uint32_t TOF_RETRY_MS = 1000;
static uint32_t tof_last_init_attempt_ms = 0;

static bool tryInitDMSSensor() {
    tof_last_init_attempt_ms = millis();
    // 100 kHz: the OpenCR core's 400 kHz timing value is unverified
    // ("BUGBUG" in its Wire.cpp), and 100 kHz is what the diagnostic
    // sketch confirmed working.
    tof.setTimeout(500);
    if (!tof.init()) {
        tof_ok = false;
        return false;
    }
    // Back-to-back continuous ranging: the chip measures on its own and
    // ReadDMSSensor() only collects finished samples, so the loop never
    // waits for a measurement (the HC-SR04's pulseIn blocked up to 25 ms).
    tof.startContinuous(0);
    tof_ok = true;
    tof_last_sample_ms = millis();
    return true;
}

static void initDMSSensor() {
    Wire.begin();
    Wire.setClock(100000);
    if (!tryInitDMSSensor()) {
        Serial.println("VL53L0X not found on I2C, dms reads -2, retrying every 1 s");
    }
}

// Non-blocking: returns the newest sample in cm, -1.0 if invalid/stale,
// -2.0 if the sensor is not initialised (init is retried once a second).
float ReadDMSSensor() {
    if (!tof_ok) {
        if (millis() - tof_last_init_attempt_ms > TOF_RETRY_MS) {
            tryInitDMSSensor();
        }
        if (!tof_ok) {
            return TOF_NOT_READY;
        }
    }

    uint8_t int_status = tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if (tof.last_status != 0) {
        // I2C failure: the value is unknown, not unchanged.
        tof_last_cm = -1.0f;
        return tof_last_cm;
    }

    if ((int_status & 0x07) != 0) {
        // A new measurement is ready.
        uint16_t mm = tof.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
        tof.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
        if (tof.last_status != 0 || mm == 0 || mm >= TOF_INVALID_MM) {
            tof_last_cm = -1.0f;
        } else {
            tof_last_cm = mm / 10.0f;
        }
        tof_last_sample_ms = millis();
    } else if (millis() - tof_last_sample_ms > TOF_STALE_MS) {
        // Sensor stopped producing samples: do not ship an old value as live.
        tof_last_cm = -1.0f;
    }

    return tof_last_cm;
}

void MecanumbotCore::begin() {
    Serial.begin(1000000);

    pinMode(BDPIN_GPIO_17, OUTPUT); 
    pinMode(BDPIN_GPIO_13, OUTPUT);
    pinMode(BDPIN_GPIO_9, OUTPUT);
    pinMode(BDPIN_GPIO_5, OUTPUT);

    digitalWrite(BDPIN_GPIO_17, LOW); //GND to 0
    digitalWrite(BDPIN_GPIO_13, LOW); //CMD received
    digitalWrite(BDPIN_GPIO_9, LOW); //XM motor write
    digitalWrite(BDPIN_GPIO_5, LOW); //AX motor write

    portHandler       = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandlerAX   = dynamixel::PacketHandler::getPacketHandler(AX_PROTOCOL_VERSION);
    packetHandlerXM   = dynamixel::PacketHandler::getPacketHandler(XM_PROTOCOL_VERSION);
    groupSyncWriteXM  = new dynamixel::GroupSyncWrite(portHandler, packetHandlerXM, XM_ADDR_GOAL_VELOCITY, XM_LEN_GOAL_VELOCITY);
    groupSyncWriteAX  = new dynamixel::GroupSyncWrite(portHandler, packetHandlerAX, AX_ADDR_GOAL_POSITION, 2);

    // Initialize GroupSyncReads
    groupSyncReadVel = new dynamixel::GroupSyncRead(portHandler, packetHandlerXM, XM_ADDR_PRESENT_VELOCITY, 4);
    groupSyncReadVel->addParam(ID_WHEEL_BL);
    groupSyncReadVel->addParam(ID_WHEEL_BR);
    groupSyncReadVel->addParam(ID_WHEEL_FL);
    groupSyncReadVel->addParam(ID_WHEEL_FR);

    groupSyncReadPos = new dynamixel::GroupSyncRead(portHandler, packetHandlerXM, XM_ADDR_PRESENT_POSITION, 4);
    groupSyncReadPos->addParam(ID_WHEEL_BL);
    groupSyncReadPos->addParam(ID_WHEEL_BR);
    groupSyncReadPos->addParam(ID_WHEEL_FL);
    groupSyncReadPos->addParam(ID_WHEEL_FR);

    initDXLConnection();    
    initializeMotors();
    sensors.init();
    sensors.initIMU();
    sensors.calibrationGyro();
    initDMSSensor();
    delay(1000);
    sensors.makeMelody(7); // SMOKE_ON_THE_WATER
}

// Sensor data structure, "should" contains every information that the ROS2 host needs
struct __attribute__((packed)) SensorData {
    //command velocity
    int16_t cmd_vel_BL = 0;
    int16_t cmd_vel_BR = 0;
    int16_t cmd_vel_FL = 0;
    int16_t cmd_vel_FR = 0;
    //goal velocities for wheels  
    int16_t vel_BL = 0;
    int16_t vel_BR = 0;
    int16_t vel_FL = 0;
    int16_t vel_FR = 0;
    //positions for wheels
    int16_t pos_BL = 0;
    int16_t pos_BR = 0;
    int16_t pos_FL = 0;
    int16_t pos_FR = 0;
    //present currents for wheels
    int16_t curr_BL = 0;
    int16_t curr_BR = 0;
    int16_t curr_FL = 0;
    int16_t curr_FR = 0;
    //accelerations for wheels
    int16_t acc_BL = 0;
    int16_t acc_BR = 0;
    int16_t acc_FL = 0;
    int16_t acc_FR = 0;
    //goal positions for neck and grabbers
    int16_t pos_N = 0;
    int16_t pos_GL = 0;
    int16_t pos_GR = 0;
    // error messages from wheels
    int16_t err_BL = 0;
    int16_t err_BR = 0;
    int16_t err_FL = 0;
    int16_t err_FR = 0;
    // distance data from DMS (VL53L0X), cm; -1 = no valid reading, -2 = sensor not initialised
    float dms = 0.0;
    //sensory data
    float voltage = 0.0;
    //IMU data
    float imu_angular_vel_x = 0.0;
    float imu_angular_vel_y = 0.0;
    float imu_angular_vel_z = 0.0;
    float imu_linear_acc_x = 0.0;
    float imu_linear_acc_y = 0.0;
    float imu_linear_acc_z = 0.0;
    float imu_magnetic_x = 0.0;
    float imu_magnetic_y = 0.0;
    float imu_magnetic_z = 0.0;
    float orientation_w = 0.0;
    float orientation_x = 0.0;
    float orientation_y = 0.0;
    float orientation_z = 0.0;
    
};

SensorData sensorData;

// err_* sentinel for "this wheel did not answer". Hardware Error Status is a
// byte, so no real error code can collide with a negative value.
#define DXL_UNREADABLE (-1)

static void markWheelsUnreadable() {
  sensorData.err_BL = DXL_UNREADABLE;
  sensorData.err_BR = DXL_UNREADABLE;
  sensorData.err_FL = DXL_UNREADABLE;
  sensorData.err_FR = DXL_UNREADABLE;
}

// CRC8 (poly 0x07) for packet integrity
static uint8_t crc8_ccitt(const uint8_t *data, size_t len){
  //crc bit
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

// Packet format sent to host: magic(2) + seq(1) + payload(SensorData) + crc(1)
struct __attribute__((packed)) SensorPacket {
  uint16_t magic;
  uint8_t seq;
  SensorData payload;
  uint8_t crc;
};

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

static uint8_t sensor_seq_counter = 0;
static uint32_t last_command_time = 0;
static int loop_count = 0;

ControlData controlData;
const int controlDataSize = sizeof(ControlData); 

void MecanumbotCore::run() {
    sensors.updateIMU();
    sensors.onMelody();
    
    // Check if we have data. 
    // and only apply the latest one.
    int availableBytes = Serial.available();
    bool received_new_cmd = false;

    // While we have enough bytes for at least one packet...
    while(availableBytes >= controlDataSize) {
        // Read the packet
        size_t bytesRead = Serial.readBytes((char*)&controlData, controlDataSize);
        if (bytesRead == controlDataSize) {
            received_new_cmd = true;
            last_command_time = millis(); // Reset safety timer
        }
        // Update available bytes for the while loop
        availableBytes = Serial.available();
    }

    if (received_new_cmd) {
        // Only apply the VERY LAST command processed in the while loop
        digitalWrite(BDPIN_GPIO_13, HIGH); //CMD received
        set_WheelVelocities(controlData.vel_BL, controlData.vel_BR, controlData.vel_FL, controlData.vel_FR);
        digitalWrite(BDPIN_GPIO_9, HIGH); //XM motor write
        set_AXPositions(controlData.pos_N, controlData.pos_GL, controlData.pos_GR);
        digitalWrite(BDPIN_GPIO_5, HIGH); //AX motor write
        
        // Update sensor reflection
        sensorData.cmd_vel_BL = controlData.vel_BL;
        sensorData.cmd_vel_BR = controlData.vel_BR;
        sensorData.cmd_vel_FL = controlData.vel_FL;
        sensorData.cmd_vel_FR = controlData.vel_FR;
        sensorData.pos_N = controlData.pos_N;
        sensorData.pos_GL = controlData.pos_GL;
        sensorData.pos_GR = controlData.pos_GR;
       
        delay(100);
        digitalWrite(BDPIN_GPIO_13, LOW); //CMD received, pin
        digitalWrite(BDPIN_GPIO_9, LOW); //XM motor write
        digitalWrite(BDPIN_GPIO_5, LOW); //AX motor write

    }
    sensorData.dms = ReadDMSSensor();
    sensorData.voltage = sensors.checkVoltage();

    // A failed read means the value is UNKNOWN, not unchanged. Holding the last
    // good sample and shipping it as live telemetry is what makes a silent
    // Dynamixel bus look like a control fault: the cmd_vel_* echo above keeps
    // tracking the host while every read-back freezes, so the two disagree and
    // nothing says why. Every read below therefore reports its own failure.
    //
    // GroupSyncRead::rxPacket() fails as a whole if any one registered ID stays
    // silent, so the four wheels are flagged together -- that is the
    // granularity a sync read gives us, not a choice made here.

    // 1. Send Sync Read Command for Velocity
    int dxl_comm_result = groupSyncReadVel->txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS) {
        if (groupSyncReadVel->isAvailable(ID_WHEEL_BL, XM_ADDR_PRESENT_VELOCITY, 4))
            sensorData.vel_BL = (int32_t)groupSyncReadVel->getData(ID_WHEEL_BL, XM_ADDR_PRESENT_VELOCITY, 4);
        if (groupSyncReadVel->isAvailable(ID_WHEEL_BR, XM_ADDR_PRESENT_VELOCITY, 4))
            sensorData.vel_BR = (int32_t)groupSyncReadVel->getData(ID_WHEEL_BR, XM_ADDR_PRESENT_VELOCITY, 4);
        if (groupSyncReadVel->isAvailable(ID_WHEEL_FL, XM_ADDR_PRESENT_VELOCITY, 4))
            sensorData.vel_FL = (int32_t)groupSyncReadVel->getData(ID_WHEEL_FL, XM_ADDR_PRESENT_VELOCITY, 4);
        if (groupSyncReadVel->isAvailable(ID_WHEEL_FR, XM_ADDR_PRESENT_VELOCITY, 4))
            sensorData.vel_FR = (int32_t)groupSyncReadVel->getData(ID_WHEEL_FR, XM_ADDR_PRESENT_VELOCITY, 4);
    } else {
        // Wheel velocity is integrated into odometry on the host. A frozen
        // non-zero sample fabricates motion for as long as the bus stays down,
        // so this is the one field zeroed rather than held: unknown speed is
        // reported as no speed, and err_* carries the reason.
        sensorData.vel_BL = 0;
        sensorData.vel_BR = 0;
        sensorData.vel_FL = 0;
        sensorData.vel_FR = 0;
        markWheelsUnreadable();
    }
    // 2. Repeat for Position with Safety Checks
    dxl_comm_result = groupSyncReadPos->txRxPacket();
    if (dxl_comm_result == COMM_SUCCESS) {
        if (groupSyncReadPos->isAvailable(ID_WHEEL_BL, XM_ADDR_PRESENT_POSITION, 4))
            sensorData.pos_BL = (int32_t)groupSyncReadPos->getData(ID_WHEEL_BL, XM_ADDR_PRESENT_POSITION, 4);
        if (groupSyncReadPos->isAvailable(ID_WHEEL_BR, XM_ADDR_PRESENT_POSITION, 4))
            sensorData.pos_BR = (int32_t)groupSyncReadPos->getData(ID_WHEEL_BR, XM_ADDR_PRESENT_POSITION, 4);
        if (groupSyncReadPos->isAvailable(ID_WHEEL_FL, XM_ADDR_PRESENT_POSITION, 4))
            sensorData.pos_FL = (int32_t)groupSyncReadPos->getData(ID_WHEEL_FL, XM_ADDR_PRESENT_POSITION, 4);
        if (groupSyncReadPos->isAvailable(ID_WHEEL_FR, XM_ADDR_PRESENT_POSITION, 4))
            sensorData.pos_FR = (int32_t)groupSyncReadPos->getData(ID_WHEEL_FR, XM_ADDR_PRESENT_POSITION, 4);
    } else {
        // Position is absolute, so a held sample is a smaller lie than a jump
        // to zero would be. It is left alone and flagged instead.
        markWheelsUnreadable();
    }
    if (loop_count % 20 == 0) {
      // One motor at a time here, so each read carries its own result and is
      // checked on its own. Current and acceleration keep their last good
      // sample on failure; err_* below is what says the sample is old.
      uint16_t curr_buffer = 0;

      if (packetHandlerXM->read2ByteTxRx(portHandler, ID_WHEEL_BL, XM_ADDR_PRESENT_CURRENT, &curr_buffer, nullptr) == COMM_SUCCESS)
          sensorData.curr_BL = (int16_t)curr_buffer;
      if (packetHandlerXM->read2ByteTxRx(portHandler, ID_WHEEL_BR, XM_ADDR_PRESENT_CURRENT, &curr_buffer, nullptr) == COMM_SUCCESS)
          sensorData.curr_BR = (int16_t)curr_buffer;
      if (packetHandlerXM->read2ByteTxRx(portHandler, ID_WHEEL_FL, XM_ADDR_PRESENT_CURRENT, &curr_buffer, nullptr) == COMM_SUCCESS)
          sensorData.curr_FL = (int16_t)curr_buffer;
      if (packetHandlerXM->read2ByteTxRx(portHandler, ID_WHEEL_FR, XM_ADDR_PRESENT_CURRENT, &curr_buffer, nullptr) == COMM_SUCCESS)
          sensorData.curr_FR = (int16_t)curr_buffer;

      uint32_t temp_buffer = 0;

      if (packetHandlerXM->read4ByteTxRx(portHandler, ID_WHEEL_BL, XM_ADDR_PROFILE_ACCELERATION, &temp_buffer, nullptr) == COMM_SUCCESS)
          sensorData.acc_BL = (int16_t)temp_buffer;
      if (packetHandlerXM->read4ByteTxRx(portHandler, ID_WHEEL_BR, XM_ADDR_PROFILE_ACCELERATION, &temp_buffer, nullptr) == COMM_SUCCESS)
          sensorData.acc_BR = (int16_t)temp_buffer;
      if (packetHandlerXM->read4ByteTxRx(portHandler, ID_WHEEL_FL, XM_ADDR_PROFILE_ACCELERATION, &temp_buffer, nullptr) == COMM_SUCCESS)
          sensorData.acc_FL = (int16_t)temp_buffer;
      if (packetHandlerXM->read4ByteTxRx(portHandler, ID_WHEEL_FR, XM_ADDR_PROFILE_ACCELERATION, &temp_buffer, nullptr) == COMM_SUCCESS)
          sensorData.acc_FR = (int16_t)temp_buffer;

      // Read through a byte of its own and assigned whole: Hardware Error
      // Status is one byte and err_* is two, so writing it in place would leave
      // the high byte of the previous value -- and the previous value is now
      // sometimes DXL_UNREADABLE.
      uint8_t err_buffer = 0;

      sensorData.err_BL = (packetHandlerXM->read1ByteTxRx(portHandler, ID_WHEEL_BL, XM_ADDR_ERROR_STATUS, &err_buffer, nullptr) == COMM_SUCCESS)
          ? (int16_t)err_buffer : (int16_t)DXL_UNREADABLE;
      sensorData.err_BR = (packetHandlerXM->read1ByteTxRx(portHandler, ID_WHEEL_BR, XM_ADDR_ERROR_STATUS, &err_buffer, nullptr) == COMM_SUCCESS)
          ? (int16_t)err_buffer : (int16_t)DXL_UNREADABLE;
      sensorData.err_FL = (packetHandlerXM->read1ByteTxRx(portHandler, ID_WHEEL_FL, XM_ADDR_ERROR_STATUS, &err_buffer, nullptr) == COMM_SUCCESS)
          ? (int16_t)err_buffer : (int16_t)DXL_UNREADABLE;
      sensorData.err_FR = (packetHandlerXM->read1ByteTxRx(portHandler, ID_WHEEL_FR, XM_ADDR_ERROR_STATUS, &err_buffer, nullptr) == COMM_SUCCESS)
          ? (int16_t)err_buffer : (int16_t)DXL_UNREADABLE;
    }

    float* tmp;

    // IMU data --> sensorData
    tmp = sensors.getImuAngularVelocity(); //static float angular_vel[3];
    sensorData.imu_angular_vel_x = tmp[0];
    sensorData.imu_angular_vel_y = tmp[1];
    sensorData.imu_angular_vel_z = tmp[2];

    tmp = sensors.getImuLinearAcc(); // static float linear_acc[3];
    sensorData.imu_linear_acc_x = tmp[0];
    sensorData.imu_linear_acc_y = tmp[1];
    sensorData.imu_linear_acc_z = tmp[2];

    tmp = sensors.getImuMagnetic(); // static float magnetic[3];
    sensorData.imu_magnetic_x = tmp[0];
    sensorData.imu_magnetic_y = tmp[1];
    sensorData.imu_magnetic_z = tmp[2];

    tmp = sensors.getOrientation(); // static float orientation[4];
    sensorData.orientation_w = tmp[0];
    sensorData.orientation_x = tmp[1];
    sensorData.orientation_y = tmp[2];
    sensorData.orientation_z = tmp[3];

    SensorPacket pkt;
    pkt.magic = 0xAA55; 
    pkt.seq = sensor_seq_counter++;
    pkt.payload = sensorData;
    pkt.crc = crc8_ccitt((const uint8_t*)&pkt, sizeof(pkt) - 1);
    
    Serial.write((const uint8_t*)&pkt, sizeof(pkt));
    loop_count++;
}