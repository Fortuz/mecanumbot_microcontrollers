#ifndef MECANUMBOT_STEPPER_H
#define MECANUMBOT_STEPPER_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// NEMA17 + TMC2209 wheel drive for the Mecanumbot.
//
// Replaces the four XM430-W210-T Dynamixel wheel motors. The neck and grabber
// AX-12A motors are untouched and still run on the Dynamixel bus.
//
// The host protocol is unchanged: wheel commands still arrive as int16 values
// in Dynamixel XM430 goal-velocity units (1 unit = 0.229 rev/min). This module
// converts those units into a step rate and generates STEP/DIR pulses from a
// hardware-timer ISR, so stepping keeps running while the main loop blocks on
// Dynamixel or ultrasonic I/O.
// ---------------------------------------------------------------------------

// ---- Wheel indices (order matches the serial protocol) --------------------
#define WHEEL_BL                 0
#define WHEEL_BR                 1
#define WHEEL_FL                 2
#define WHEEL_FR                 3
#define NUM_WHEELS               4

// ---- Pin map --------------------------------------------------------------
// GPIO 5, 9, 13 and 17 stay reserved for the oscilloscope timing rig, and
// Arduino pins 2/3 stay reserved for the ultrasonic sensor.
#define PIN_STEP_BL              BDPIN_GPIO_1
#define PIN_DIR_BL               BDPIN_GPIO_2
#define PIN_STEP_BR              BDPIN_GPIO_3
#define PIN_DIR_BR               BDPIN_GPIO_4
#define PIN_STEP_FL              BDPIN_GPIO_6
#define PIN_DIR_FL               BDPIN_GPIO_7
#define PIN_STEP_FR              BDPIN_GPIO_8
#define PIN_DIR_FR               BDPIN_GPIO_10
#define PIN_STEPPER_ENABLE       BDPIN_GPIO_11   // shared /EN, active LOW

// ---- Mechanics ------------------------------------------------------------
#define STEPPER_FULL_STEPS_PER_REV   200         // 1.8 deg NEMA17
#define STEPPER_MICROSTEPS           16          // MS1/MS2 straps, or UART
#define STEPPER_GEAR_RATIO           1.0f        // motor rev per wheel rev
#define STEPPER_STEPS_PER_WHEEL_REV \
  ((int32_t)(STEPPER_FULL_STEPS_PER_REV * STEPPER_MICROSTEPS * STEPPER_GEAR_RATIO))

// Per-wheel direction sign. Flip these instead of swapping motor wiring.
#define STEPPER_INVERT_BL        0
#define STEPPER_INVERT_BR        1
#define STEPPER_INVERT_FL        0
#define STEPPER_INVERT_FR        1

// TMC2209 direction polarity: 1 if DIR HIGH means the positive wheel direction.
#define STEPPER_DIR_HIGH_IS_FORWARD 1

// ---- Command units --------------------------------------------------------
// One Dynamixel XM430 goal-velocity unit is 0.229 rev/min. Keeping this scale
// means the ROS 2 host and test_cr.py need no change.
#define DXL_VELOCITY_UNIT_RPM    0.229f
#define DXL_VELOCITY_UNIT_RPS    (DXL_VELOCITY_UNIT_RPM / 60.0f)

// Reject commands past this (in Dynamixel units) so a bad packet cannot ask
// for a rate the motors will simply stall at. 330 units ~= 75 rev/min.
#define STEPPER_MAX_DXL_UNITS    330

// ---- Timing ---------------------------------------------------------------
// 20 kHz ISR: one step per tick at most, so 20000 steps/s per wheel. At 1/16
// microstepping that is 375 rev/min, far above anything the robot drives at.
#define STEPPER_ISR_HZ           20000
#define STEPPER_ISR_PERIOD_US    (1000000UL / STEPPER_ISR_HZ)

// Ramp is evaluated at ISR_HZ / RAMP_DIVIDER. Steppers stall on an instant
// jump to speed, so this replaces the XM430's internal profile acceleration.
#define STEPPER_RAMP_DIVIDER     20              // -> 1 kHz
#define STEPPER_RAMP_HZ          (STEPPER_ISR_HZ / STEPPER_RAMP_DIVIDER)
#define STEPPER_ACCEL_STEPS_S2   24000L          // steps/s^2

// Stop the wheels if the host goes quiet. The Dynamixels held their last goal
// velocity forever; an open-loop stepper drive must not.
#define STEPPER_CMD_TIMEOUT_MS   500

// Set to 1 to configure the drivers over the TMC2209 single-wire UART
// (run current, microsteps, StallGuard). Requires the TMCStepper library and
// the four MS1/MS2 address straps. Leave at 0 for standalone STEP/DIR mode.
#define STEPPER_USE_UART         0

#if STEPPER_USE_UART
  #define STEPPER_UART_PORT      Serial2
  #define STEPPER_RSENSE         0.11f           // most TMC2209 breakouts
  #define STEPPER_RUN_CURRENT_MA 1000
  #define STEPPER_HOLD_MULTIPLIER 0.4f
  #define STEPPER_STALL_THRESHOLD 60
#endif

// ---- Driver status bits reported in the telemetry err_* fields -------------
#define STEPPER_ERR_NONE         0x00
#define STEPPER_ERR_STALL        0x01
#define STEPPER_ERR_OVERTEMP_WARN 0x02
#define STEPPER_ERR_OVERTEMP_SHUT 0x04
#define STEPPER_ERR_SHORT        0x08
#define STEPPER_ERR_OPEN_LOAD    0x10
#define STEPPER_ERR_CMD_TIMEOUT  0x20
#define STEPPER_ERR_CLAMPED      0x40
#define STEPPER_ERR_UART_FAIL    0x80

namespace MecanumbotStepper {

  // Bring up pins, the driver UART (if enabled) and the step timer.
  void begin();

  // --- the velocity -> step conversion the whole module is built around ---
  // Dynamixel goal-velocity units -> signed steps per second.
  float stepsPerSecondFromDxlUnits(int16_t dxl_units);
  // and back, for telemetry.
  int16_t dxlUnitsFromStepsPerSecond(float steps_per_second);

  // Apply a wheel command set. Units and argument order are identical to the
  // set_WheelVelocities() call this replaces.
  void setWheelVelocities(int16_t vBL, int16_t vBR, int16_t vFL, int16_t vFR);

  // Ramp everything to a standstill (does not drop /EN).
  void stopAll();

  // Driver output stage. Disabling frees the motors but also removes holding
  // torque, so the robot can be pushed or roll on a slope.
  void enable(bool on);
  bool isEnabled();

  // Call every main-loop pass: services the command watchdog and, when UART is
  // enabled, polls DRV_STATUS.
  void service();

  // Reset the command watchdog after a valid host packet.
  void noteCommandReceived();

  // --- telemetry sources, filling the fields the XM430 used to report ---
  // Ramped step rate converted back to Dynamixel units.
  void getMeasuredVelocities(int16_t out[NUM_WHEELS]);
  // Integrated step count in XM430-style 4096-ticks-per-revolution units.
  void getPositionTicks(int16_t out[NUM_WHEELS]);
  // Raw signed microsteps since boot. This is the honest odometry source.
  void getStepCounts(int32_t out[NUM_WHEELS]);
  // Configured ramp rate, in Dynamixel-equivalent units per second.
  void getAccelUnits(int16_t out[NUM_WHEELS]);
  // Driver/health bits, see STEPPER_ERR_* above.
  void getDriverStatus(int16_t out[NUM_WHEELS]);
  // StallGuard load result when UART is enabled, otherwise 0.
  void getLoadResult(int16_t out[NUM_WHEELS]);
}

#endif
