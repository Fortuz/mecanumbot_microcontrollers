#include "mecanumbot.h"


dynamixel::PortHandler *portHandler       = nullptr;
dynamixel::PacketHandler *packetHandlerAX = nullptr;
dynamixel::PacketHandler *packetHandlerXM = nullptr;
dynamixel::GroupSyncWrite *groupSyncWriteXM = nullptr;
dynamixel::GroupSyncWrite *groupSyncWriteAX = nullptr;


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
  // Initialize motors here if needed
  // Test back left wheel
    initializeXMMotors(ID_WHEEL_BL);
    initializeXMMotors(ID_WHEEL_BR);
    initializeXMMotors(ID_WHEEL_FL);
    initializeXMMotors(ID_WHEEL_FR);

    writeByte(packetHandlerAX, ID_GRABBER_R, AX_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    writeByte(packetHandlerAX, ID_GRABBER_L, AX_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
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

    portHandler       = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandlerAX   = dynamixel::PacketHandler::getPacketHandler(AX_PROTOCOL_VERSION);
    packetHandlerXM   = dynamixel::PacketHandler::getPacketHandler(XM_PROTOCOL_VERSION);
    groupSyncWriteXM  = new dynamixel::GroupSyncWrite(portHandler, packetHandlerXM, XM_ADDR_GOAL_VELOCITY, XM_LEN_GOAL_VELOCITY);
    // 2 is probably the length of the AX goal position
    groupSyncWriteAX  = new dynamixel::GroupSyncWrite(portHandler, packetHandlerAX, AX_ADDR_GOAL_POSITION, 2);


    initDXLConnection();    

    // Test the motors
    initializeMotors();

    // Wait a second for everything to settle
    delay(1000);

}

struct ControlData {
  int16_t vel_BL = 0;
  int16_t vel_BR = 0;
  int16_t vel_FL = 0;
  int16_t vel_FR = 0;
  int16_t pos_N = 0;
  int16_t pos_GL = 0;
  int16_t pos_GR = 0;
};

ControlData controlData;

const int controlDataSize = sizeof(ControlData);

void MecanumbotCore::run() {
    //Serial.println(portHandler->getPortName());

    if (Serial.available() > 0)
    {
        Serial.readBytes((char*)&controlData, controlDataSize);
        //TODO: Add constraints
        //TODO: calculate checksum and verify the message
        set_WheelVelocities(controlData.vel_BL, controlData.vel_BR, controlData.vel_FL, controlData.vel_FR);
        set_AXPositions(controlData.pos_N, controlData.pos_GL, controlData.pos_GR);

        // Print the contents of controlData on serial as a struct
        // This sends the raw binary data of the struct, not text
        
    }
    
    Serial.write((uint8_t*)&controlData, sizeof(controlData));
}
