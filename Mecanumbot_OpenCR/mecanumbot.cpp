#include "mecanumbot.h"

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

void MecanumbotCore::begin() {
    Serial.begin(1000000); 


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
        set_WheelVelocities(controlData.vel_BL, controlData.vel_BR, controlData.vel_FL, controlData.vel_FR);
        set_AXPositions(controlData.pos_N, controlData.pos_GL, controlData.pos_GR);
        
        // Update sensor reflection
        sensorData.cmd_vel_BL = controlData.vel_BL;
        sensorData.cmd_vel_BR = controlData.vel_BR;
        sensorData.cmd_vel_FL = controlData.vel_FL;
        sensorData.cmd_vel_FR = controlData.vel_FR;
        sensorData.pos_N = controlData.pos_N;
        sensorData.pos_GL = controlData.pos_GL;
        sensorData.pos_GR = controlData.pos_GR;
    }

    // SAFETY TIMEOUT: If no data from Pi for 200ms, STOP ROBOT
    if (millis() - last_command_time > 200) {
        set_WheelVelocities(0, 0, 0, 0);
    }
    // ------------------------------------------

    sensorData.voltage = sensors.checkVoltage();

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
    }

    if (loop_count % 20 == 0) {
      //present currents for wheels
      packetHandlerXM->read2ByteTxRx(portHandler, ID_WHEEL_BL, XM_ADDR_PRESENT_CURRENT, (uint16_t*)&sensorData.curr_BL, nullptr);
      packetHandlerXM->read2ByteTxRx(portHandler, ID_WHEEL_BR, XM_ADDR_PRESENT_CURRENT, (uint16_t*)&sensorData.curr_BR, nullptr);
      packetHandlerXM->read2ByteTxRx(portHandler, ID_WHEEL_FL, XM_ADDR_PRESENT_CURRENT, (uint16_t*)&sensorData.curr_FL, nullptr);
      packetHandlerXM->read2ByteTxRx(portHandler, ID_WHEEL_FR, XM_ADDR_PRESENT_CURRENT, (uint16_t*)&sensorData.curr_FR, nullptr);

      //accelerations for wheels
      packetHandlerXM->read4ByteTxRx(portHandler, ID_WHEEL_BL, XM_ADDR_PROFILE_ACCELERATION, (uint32_t*)&sensorData.acc_BL, nullptr);
      packetHandlerXM->read4ByteTxRx(portHandler, ID_WHEEL_BR, XM_ADDR_PROFILE_ACCELERATION, (uint32_t*)&sensorData.acc_BR, nullptr);
      packetHandlerXM->read4ByteTxRx(portHandler, ID_WHEEL_FL, XM_ADDR_PROFILE_ACCELERATION, (uint32_t*)&sensorData.acc_FL, nullptr);
      packetHandlerXM->read4ByteTxRx(portHandler, ID_WHEEL_FR, XM_ADDR_PROFILE_ACCELERATION, (uint32_t*)&sensorData.acc_FR, nullptr); 
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