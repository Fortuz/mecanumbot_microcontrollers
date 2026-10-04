#ifndef MECANUMBOT_H
#define MECANUMBOT_H

#include <stdint.h>
#include <DynamixelSDK.h>
#include "mecanumbot_sensor.h"
#include "mecanumbot_stepper.h"

// ---------------------------------------------------------------------------
// Stepper-wheel build of the Mecanumbot OpenCR firmware.
//
// Wheels:            4x NEMA17 driven by TMC2209 in STEP/DIR   -> mecanumbot_stepper
// Neck and grabbers: 3x AX-12A on the Dynamixel bus, protocol 1 -> here
//
// Because the XM430 wheel motors are gone, the Dynamixel bus now carries a
// single protocol. The protocol-1/protocol-2 switching the old firmware had to
// do on every wheel write is no longer needed.
//
// The host wire format is byte-for-byte identical to the Dynamixel build.
// ---------------------------------------------------------------------------

#define DEVICENAME               "OpenCR_DXL_Port"
#define BAUDRATE                 1000000
#define VOLTAGE_PIN              16                 // ADC0 on OpenCR

#define TORQUE_ENABLE            1
#define TORQUE_DISABLE           0

#define ID_NECK                  7
#define ID_GRABBER_L             6
#define ID_GRABBER_R             5

#define OPENED                   1
#define CLOSED                   0

// Mecanum drive working parameters (hand measured)
#define WHEEL_SEPARATION_X       0.13f   // 13 cm
#define WHEEL_SEPARATION_Y       0.305f  // 30.5 cm
#define WHEEL_RADIUS             0.065f  // 6.5 cm

// AX-12A motor parameters (neck and grabbers) - position control
// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
#define AX_ADDR_TORQUE_ENABLE        24 // RW, 1 byte
#define AX_ADDR_GOAL_POSITION        30 // RW, 2 bytes
#define AX_ADDR_MOVING_SPEED         32 // RW, 2 bytes
#define AX_ADDR_PRESENT_POSITION     36 // R,  2 bytes
#define AX_ADDR_PRESENT_SPEED        38 // R,  2 bytes
#define AX_ADDR_PRESENT_VOLTAGE      42 // R,  1 byte
#define AX_ADDR_PRESENT_TEMPERATURE  43 // R,  1 byte

#define AX_PROTOCOL_VERSION          1.0
#define AX_LEN_GOAL_POSITION         2

#define NECK_MIN_POSITION_VALUE      200
#define NECK_MAX_POSITION_VALUE      860
#define GRABBER_MIN_POSITION_VALUE   160
#define GRABBER_FRONT_POSITION_VALUE 512
#define GRABBER_MAX_POSITION_VALUE   854

// Telemetry cadence. The Dynamixel build was throttled incidentally by its two
// blocking wheel sync-reads; those are gone, so the rate is now explicit and
// the host sees roughly the packet rate it saw before.
#define TELEMETRY_HZ                 100

// Oscilloscope timing rig: the old firmware held these high with delay(100),
// which is not acceptable when a step ISR is running. The pulse is now timed
// from the main loop instead. The logger in oscilloscope/ polls every ~4 us.
#define DIAG_PULSE_MS                2

namespace MecanumbotCore {
  void begin();
  void run();
}

#endif
