/**
 * AX-12A minimal driver (Protocol 1.0) sharing the same Dynamixel2Arduino bus
 * with the wheel XM motors (Protocol 2.0). We switch protocol around each AX op
 * and restore to 2.0 immediately to avoid interfering with wheel control.
 */

#ifndef CAMERA_GIMBAL_DRIVER_H_
#define CAMERA_GIMBAL_DRIVER_H_

#include <Dynamixel2Arduino.h>

class CameraGimbalDriver {
public:
	explicit CameraGimbalDriver(Dynamixel2Arduino &dxl_ref);

	bool init();
	bool is_connected();

	bool setTorque(bool on);
	bool writeGoal(uint8_t id, uint16_t pos);
	bool readPresent(uint8_t id, uint16_t &pos);

private:
	// AX-12A (Protocol 1.0) control table
	static constexpr float AX_PROTOCOL_VERSION = 1.0f;
	static constexpr float XM_PROTOCOL_VERSION = 2.0f;
	static constexpr uint8_t AX_REG_TORQUE_ENABLE = 24;     // 1 byte
	static constexpr uint8_t AX_REG_GOAL_POSITION = 30;     // 2 bytes
	static constexpr uint8_t AX_REG_PRESENT_POSITION = 36;  // 2 bytes

	Dynamixel2Arduino &dxl;
	bool inited_ = false;
	bool connected_ = false;

	// IDs: Neck=7, Left=6, Right=5
	uint8_t id_neck_ = 7;
	uint8_t id_left_ = 6;
	uint8_t id_right_ = 5;

	void setProtoV1_();
	void restoreProtoV2_();
};

#endif // CAMERA_GIMBAL_DRIVER_H_
