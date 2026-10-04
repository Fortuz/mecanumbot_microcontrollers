#include <Arduino.h>
#include "mecanumbot.h"

// ---------------------------------------------------------------------------
// Dynamixel handles. Protocol 1 only now: the wheels moved to STEP/DIR.
// ---------------------------------------------------------------------------
dynamixel::PortHandler   *portHandler      = nullptr;
dynamixel::PacketHandler *packetHandlerAX  = nullptr;
dynamixel::GroupSyncWrite *groupSyncWriteAX = nullptr;

static MecanumbotSensor sensors;

const int TRIG = 2, ECHO = 3;

// Oscilloscope timing rig pins, unchanged from the Dynamixel build.
static const int PIN_DIAG_GND      = BDPIN_GPIO_17;
static const int PIN_DIAG_CMD      = BDPIN_GPIO_13;   // command received
static const int PIN_DIAG_WHEEL    = BDPIN_GPIO_9;    // wheel command applied
static const int PIN_DIAG_AX       = BDPIN_GPIO_5;    // AX motor write

static void writeAXByte(int id, int address, uint8_t data) {
  uint8_t dxl_error = 0;
  int result = packetHandlerAX->write1ByteTxRx(portHandler, id, address, data, &dxl_error);
  if (result != COMM_SUCCESS) {
    Serial.print("COMM Error: ");
    Serial.println(packetHandlerAX->getTxRxResult(result));
  } else if (dxl_error != 0) {
    Serial.print("Error Code: ");
    Serial.println(packetHandlerAX->getRxPacketError(dxl_error));
  }
}

// The wheels are steppers now; this is the drop-in replacement for the old
// set_WheelVelocities(), same argument order and same Dynamixel velocity units.
static void set_WheelVelocities(int16_t vBL, int16_t vBR, int16_t vFL, int16_t vFR) {
  MecanumbotStepper::setWheelVelocities(vBL, vBR, vFL, vFR);
}

static void set_AXPositions(int16_t pN, int16_t pGL, int16_t pGR) {
  // AX-12A goal position is a 2-byte register.
  uint8_t param_GL[2] = { DXL_LOBYTE(pGL), DXL_HIBYTE(pGL) };
  uint8_t param_GR[2] = { DXL_LOBYTE(pGR), DXL_HIBYTE(pGR) };
  uint8_t param_N[2]  = { DXL_LOBYTE(pN),  DXL_HIBYTE(pN)  };

  groupSyncWriteAX->addParam(ID_GRABBER_L, param_GL);
  groupSyncWriteAX->addParam(ID_GRABBER_R, param_GR);
  groupSyncWriteAX->addParam(ID_NECK, param_N);
  groupSyncWriteAX->txPacket();
  groupSyncWriteAX->clearParam();
}

static void initializeMotors() {
  writeAXByte(ID_GRABBER_R, AX_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  writeAXByte(ID_GRABBER_L, AX_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  writeAXByte(ID_NECK,      AX_ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
}

static void initDXLConnection() {
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

static float ReadUSSensor() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // 25000 microsecond timeout (~400cm max range)
  unsigned long duration = pulseIn(ECHO, HIGH, 25000);
  if (duration == 0) {
    return -1.0;
  }
  return duration * 0.034 / 2.0;
}

void MecanumbotCore::begin() {
  Serial.begin(1000000);

  // Park the stepper drivers before anything else. The TMC2209 /EN line has a
  // pulldown on most breakouts, so a floating pin during the multi-second
  // sensor init would leave the motors energized and unattended.
  pinMode(PIN_STEPPER_ENABLE, OUTPUT);
  digitalWrite(PIN_STEPPER_ENABLE, HIGH);   // active LOW

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(PIN_DIAG_GND, OUTPUT);
  pinMode(PIN_DIAG_CMD, OUTPUT);
  pinMode(PIN_DIAG_WHEEL, OUTPUT);
  pinMode(PIN_DIAG_AX, OUTPUT);

  digitalWrite(PIN_DIAG_GND, LOW);
  digitalWrite(PIN_DIAG_CMD, LOW);
  digitalWrite(PIN_DIAG_WHEEL, LOW);
  digitalWrite(PIN_DIAG_AX, LOW);

  portHandler      = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandlerAX  = dynamixel::PacketHandler::getPacketHandler(AX_PROTOCOL_VERSION);
  groupSyncWriteAX = new dynamixel::GroupSyncWrite(portHandler, packetHandlerAX,
                                                   AX_ADDR_GOAL_POSITION,
                                                   AX_LEN_GOAL_POSITION);

  initDXLConnection();
  initializeMotors();

  sensors.init();
  sensors.initIMU();
  sensors.calibrationGyro();

  // Start stepping only after the gyro calibration settles, so the drivers are
  // not holding current through a long blocking init.
  MecanumbotStepper::begin();

  delay(1000);
  sensors.makeMelody(7); // SMOKE_ON_THE_WATER
}

// Sensor data structure. Layout is unchanged from the Dynamixel build:
// 27 int16 followed by 15 float, so the host unpack format stays '<27h15f'.
struct __attribute__((packed)) SensorData {
    //command velocity
    int16_t cmd_vel_BL = 0;
    int16_t cmd_vel_BR = 0;
    int16_t cmd_vel_FL = 0;
    int16_t cmd_vel_FR = 0;
    //ramped step rate, converted back to Dynamixel velocity units
    int16_t vel_BL = 0;
    int16_t vel_BR = 0;
    int16_t vel_FL = 0;
    int16_t vel_FR = 0;
    //integrated step count, in XM430-style 4096 ticks per revolution
    int16_t pos_BL = 0;
    int16_t pos_BR = 0;
    int16_t pos_FL = 0;
    int16_t pos_FR = 0;
    //no current sense on a TMC2209 in STEP/DIR mode: StallGuard load, or 0
    int16_t curr_BL = 0;
    int16_t curr_BR = 0;
    int16_t curr_FL = 0;
    int16_t curr_FR = 0;
    //configured ramp rate, in Dynamixel velocity units per second
    int16_t acc_BL = 0;
    int16_t acc_BR = 0;
    int16_t acc_FL = 0;
    int16_t acc_FR = 0;
    //goal positions for neck and grabbers
    int16_t pos_N = 0;
    int16_t pos_GL = 0;
    int16_t pos_GR = 0;
    //driver health bits, see STEPPER_ERR_* in mecanumbot_stepper.h
    int16_t err_BL = 0;
    int16_t err_BR = 0;
    int16_t err_FL = 0;
    int16_t err_FR = 0;
    // distance data from DMS
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

// CRC8 (poly 0x07) for packet integrity
static uint8_t crc8_ccitt(const uint8_t *data, size_t len){
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

static uint8_t  sensor_seq_counter = 0;
static uint32_t last_telemetry_ms  = 0;
static uint32_t diag_pulse_start   = 0;
static bool     diag_pulse_active  = false;

ControlData controlData;
const int controlDataSize = sizeof(ControlData);

void MecanumbotCore::run() {
    sensors.updateIMU();
    sensors.onMelody();
    MecanumbotStepper::service();

    // Drain the input and keep only the newest command, exactly as before.
    // The framing is unchanged: a bare 14-byte struct, no header and no CRC.
    int availableBytes = Serial.available();
    bool received_new_cmd = false;

    while (availableBytes >= controlDataSize) {
        size_t bytesRead = Serial.readBytes((char*)&controlData, controlDataSize);
        if (bytesRead == controlDataSize) {
            received_new_cmd = true;
        }
        availableBytes = Serial.available();
    }

    if (received_new_cmd) {
        MecanumbotStepper::noteCommandReceived();

        digitalWrite(PIN_DIAG_CMD, HIGH);
        set_WheelVelocities(controlData.vel_BL, controlData.vel_BR,
                            controlData.vel_FL, controlData.vel_FR);
        digitalWrite(PIN_DIAG_WHEEL, HIGH);
        set_AXPositions(controlData.pos_N, controlData.pos_GL, controlData.pos_GR);
        digitalWrite(PIN_DIAG_AX, HIGH);

        sensorData.cmd_vel_BL = controlData.vel_BL;
        sensorData.cmd_vel_BR = controlData.vel_BR;
        sensorData.cmd_vel_FL = controlData.vel_FL;
        sensorData.cmd_vel_FR = controlData.vel_FR;
        sensorData.pos_N  = controlData.pos_N;
        sensorData.pos_GL = controlData.pos_GL;
        sensorData.pos_GR = controlData.pos_GR;

        // Non-blocking replacement for the old delay(100): the diagnostic pins
        // stay high long enough for the logger to catch the edge, without
        // stalling the command path.
        diag_pulse_start  = millis();
        diag_pulse_active = true;
    }

    if (diag_pulse_active && (millis() - diag_pulse_start) >= DIAG_PULSE_MS) {
        digitalWrite(PIN_DIAG_CMD, LOW);
        digitalWrite(PIN_DIAG_WHEEL, LOW);
        digitalWrite(PIN_DIAG_AX, LOW);
        diag_pulse_active = false;
    }

    if ((millis() - last_telemetry_ms) < (1000UL / TELEMETRY_HZ)) {
        return;
    }
    last_telemetry_ms = millis();

    sensorData.dms     = ReadUSSensor();
    sensorData.voltage = sensors.checkVoltage();

    // Wheel telemetry now comes from the step generator rather than from
    // Dynamixel sync-reads. These are commanded quantities: an open-loop
    // stepper cannot report what the wheel actually did.
    int16_t buf[NUM_WHEELS];

    MecanumbotStepper::getMeasuredVelocities(buf);
    sensorData.vel_BL = buf[WHEEL_BL];
    sensorData.vel_BR = buf[WHEEL_BR];
    sensorData.vel_FL = buf[WHEEL_FL];
    sensorData.vel_FR = buf[WHEEL_FR];

    MecanumbotStepper::getPositionTicks(buf);
    sensorData.pos_BL = buf[WHEEL_BL];
    sensorData.pos_BR = buf[WHEEL_BR];
    sensorData.pos_FL = buf[WHEEL_FL];
    sensorData.pos_FR = buf[WHEEL_FR];

    MecanumbotStepper::getLoadResult(buf);
    sensorData.curr_BL = buf[WHEEL_BL];
    sensorData.curr_BR = buf[WHEEL_BR];
    sensorData.curr_FL = buf[WHEEL_FL];
    sensorData.curr_FR = buf[WHEEL_FR];

    MecanumbotStepper::getAccelUnits(buf);
    sensorData.acc_BL = buf[WHEEL_BL];
    sensorData.acc_BR = buf[WHEEL_BR];
    sensorData.acc_FL = buf[WHEEL_FL];
    sensorData.acc_FR = buf[WHEEL_FR];

    MecanumbotStepper::getDriverStatus(buf);
    sensorData.err_BL = buf[WHEEL_BL];
    sensorData.err_BR = buf[WHEEL_BR];
    sensorData.err_FL = buf[WHEEL_FL];
    sensorData.err_FR = buf[WHEEL_FR];

    float* tmp;

    tmp = sensors.getImuAngularVelocity();
    sensorData.imu_angular_vel_x = tmp[0];
    sensorData.imu_angular_vel_y = tmp[1];
    sensorData.imu_angular_vel_z = tmp[2];

    tmp = sensors.getImuLinearAcc();
    sensorData.imu_linear_acc_x = tmp[0];
    sensorData.imu_linear_acc_y = tmp[1];
    sensorData.imu_linear_acc_z = tmp[2];

    tmp = sensors.getImuMagnetic();
    sensorData.imu_magnetic_x = tmp[0];
    sensorData.imu_magnetic_y = tmp[1];
    sensorData.imu_magnetic_z = tmp[2];

    tmp = sensors.getOrientation();
    sensorData.orientation_w = tmp[0];
    sensorData.orientation_x = tmp[1];
    sensorData.orientation_y = tmp[2];
    sensorData.orientation_z = tmp[3];

    SensorPacket pkt;
    pkt.magic = 0xAA55;
    pkt.seq   = sensor_seq_counter++;
    pkt.payload = sensorData;
    pkt.crc = crc8_ccitt((const uint8_t*)&pkt, sizeof(pkt) - 1);

    Serial.write((const uint8_t*)&pkt, sizeof(pkt));
}
