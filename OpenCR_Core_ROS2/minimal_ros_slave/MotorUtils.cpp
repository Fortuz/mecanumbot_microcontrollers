#include "MotorUtils.h"

// Pointers
dynamixel::PortHandler *portHandler       = nullptr;
dynamixel::PacketHandler *packetHandlerAX = nullptr;
dynamixel::PacketHandler *packetHandlerXM = nullptr;
dynamixel::GroupSyncWrite *groupSyncWrite = nullptr;

void init_Neck(){
  set_TorqueEnableAX(ID_NECK);
  set_NeckPosition(810);
}

void init_Grabber(){
  set_TorqueEnableAX(ID_GRABBER_L);
  set_TorqueEnableAX(ID_GRABBER_R);
  writeByte(packetHandlerAX, ID_GRABBER_R, AX_ADDR_GOAL_POSITION, GRABBER_FRONT_POSITION_VALUE); 
  writeByte(packetHandlerAX, ID_GRABBER_L, AX_ADDR_GOAL_POSITION, GRABBER_FRONT_POSITION_VALUE);
}

void init_Dynamixel(const char* device_name, uint32_t baudrate) {
  // Port and packet initialization
  portHandler       = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandlerAX   = dynamixel::PacketHandler::getPacketHandler(AX_PROTOCOL_VERSION);
  packetHandlerXM   = dynamixel::PacketHandler::getPacketHandler(XM_PROTOCOL_VERSION);
  groupSyncWrite    = new dynamixel::GroupSyncWrite(portHandler, packetHandlerXM, XM_ADDR_GOAL_VELOCITY, XM_LEN_GOAL_VELOCITY);

  if (!portHandler->openPort()) {
    Serial.println("Failed to open port");
    return;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    Serial.println("Failed to set baudrate");
    return;
  }
  Serial.println("Port handler is ready!");
}

void init_AllWheels() {
  const uint8_t XM_ID[XM_NUM_MOTORS] = {ID_WHEEL_BL, ID_WHEEL_BR, ID_WHEEL_FL, ID_WHEEL_FR};
  for (int i = 0; i < XM_NUM_MOTORS; i++) {
    init_Wheel(XM_ID[i]);
  }
}

void init_Wheel(int ID) {
  writeByte(packetHandlerXM, ID, XM_ADDR_TORQUE_ENABLE, DXL_TORQUE_DISABLE);
  writeByte(packetHandlerXM, ID, XM_ADDR_OPERATING_MODE, XM_VELOCITY_MODE);
  writeByte(packetHandlerXM, ID, XM_ADDR_TORQUE_ENABLE, DXL_TORQUE_ENABLE);
}

void set_TorqueEnableAX(int ID) {
  writeByte(packetHandlerAX, ID, AX_ADDR_TORQUE_ENABLE, DXL_TORQUE_ENABLE);
}

void set_NeckPosition(uint16_t position) {
  writeByte(packetHandlerAX, ID_NECK, AX_ADDR_GOAL_POSITION, position);
}

void set_GrabberStatus(int grabber_status) {
  if (grabber_status == CLOSED) {
    writeByte(packetHandlerAX, ID_GRABBER_R, AX_ADDR_GOAL_POSITION, 360); 
    writeByte(packetHandlerAX, ID_GRABBER_L, AX_ADDR_GOAL_POSITION, 655); 
  } else if (grabber_status == OPENED) {
    writeByte(packetHandlerAX, ID_GRABBER_L, AX_ADDR_GOAL_POSITION, 360); 
    writeByte(packetHandlerAX, ID_GRABBER_R, AX_ADDR_GOAL_POSITION, 655); 
    }
}

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

  groupSyncWrite->addParam(ID_WHEEL_BL, param_goal_velocity_BL);
  groupSyncWrite->addParam(ID_WHEEL_BR, param_goal_velocity_BR);
  groupSyncWrite->addParam(ID_WHEEL_FL, param_goal_velocity_FL);
  groupSyncWrite->addParam(ID_WHEEL_FR, param_goal_velocity_FR);
  
  groupSyncWrite->txPacket();
  groupSyncWrite->clearParam();
}