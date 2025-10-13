#include "camera_gimbal_driver.h"

CameraGimbalDriver::CameraGimbalDriver(Dynamixel2Arduino &dxl_ref)
	: dxl(dxl_ref) {}

void CameraGimbalDriver::setProtoV1_() {
	if (proto_is_v1_) return;
	dxl.setPortProtocolVersion(AX_PROTOCOL_VERSION);
	proto_is_v1_ = true;
}

void CameraGimbalDriver::restoreProtoV2_() {
	if (!proto_is_v1_) return;
	dxl.setPortProtocolVersion(XM_PROTOCOL_VERSION);
	proto_is_v1_ = false;
}

bool CameraGimbalDriver::init() {
	// Quick connectivity probe (optional)
	connected_ = is_connected();
	inited_ = true;
	return true;
}

bool CameraGimbalDriver::is_connected() {
	bool ok = true;
	setProtoV1_();
    // pings each motor ID
	ok &= dxl.ping(id_neck_);
	ok &= dxl.ping(id_left_);
	ok &= dxl.ping(id_right_);
	restoreProtoV2_();
	connected_ = ok;
	return ok;
}

bool CameraGimbalDriver::setTorqueInit(bool on) {
	if (!inited_) return false;
    bool ok = true;
    setProtoV1_();
    uint8_t v = on ? 1 : 0;
    if (on == false) {
        ok &= dxl.write(id_neck_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
		ok &= dxl.write(id_left_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
		ok &= dxl.write(id_right_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
    }
	else
	{	
        // writes 1 to set torque enable register of each motor
		ok &= dxl.write(id_neck_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
		ok &= dxl.write(id_left_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
		ok &= dxl.write(id_right_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
	}
	restoreProtoV2_();
	return ok;
}

bool CameraGimbalDriver::setTorque(bool on) {
	if (!inited_) return false;
    bool ok = true;
    //setProtoV1_();
    uint8_t v = on ? 1 : 0;
    if (on == false) {
        ok &= dxl.write(id_neck_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
		ok &= dxl.write(id_left_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
		ok &= dxl.write(id_right_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
    }
	else
	{	
        // writes 1 to set torque enable register of each motor
		ok &= dxl.write(id_neck_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
		ok &= dxl.write(id_left_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
		ok &= dxl.write(id_right_, AX_REG_TORQUE_ENABLE, &v, 1, 10);
	}
	//restoreProtoV2_();
	return ok;
}

bool CameraGimbalDriver::writeGoal(uint8_t id, uint16_t pos) {
	if (!inited_) return false;
	//setProtoV1_();
	uint16_t p = pos; // little-endian on AVR/ARM
    // writes 2 bytes to goal position register of specified motor: returns true on success
	bool ok = dxl.write(id, AX_REG_GOAL_POSITION, (uint8_t*)&p, sizeof(p), 10);
	//restoreProtoV2_();
	return ok;
}

bool CameraGimbalDriver::readPresent(uint8_t id, uint16_t &pos) {
	if (!inited_) return false;
	uint16_t value = 0; // invalid
	//setProtoV1_();
	// addr_length = 1 for Protocol 1.0 (AX series), data length = 2 bytes
    // reads 2 bytes from present position register of specified motor: returns true on success
	int32_t nread = dxl.read(id, AX_REG_PRESENT_POSITION, 2, (uint8_t *)&value, sizeof(value), 10);
	//restoreProtoV2_();
    // if 2 bytes can be read, copy to output parameter
	if (nread == (int32_t)sizeof(value)) pos = value;
	return nread == (int32_t)sizeof(value);
}
