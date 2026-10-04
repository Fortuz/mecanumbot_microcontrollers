#include <Arduino.h>
#include <math.h>
#include "mecanumbot_stepper.h"

#if STEPPER_USE_UART
  #include <TMCStepper.h>
#endif

// ---------------------------------------------------------------------------
// Step generation
//
// A 20 kHz hardware-timer ISR runs a phase accumulator per wheel. Each wheel
// holds a signed step rate in whole steps/s; the ISR adds a precomputed Q24
// phase increment every tick and emits one STEP pulse per accumulator
// overflow. The expensive rate -> increment conversion happens on the 1 kHz
// ramp tick, not in the fast path.
//
// The ISR owns the STEP/DIR pins and the step counters. The main loop only
// writes target rates and reads counters; both are single 32-bit words, which
// are read and written atomically on the Cortex-M7, so no critical section is
// needed for them.
// ---------------------------------------------------------------------------

#define PHASE_ONE_STEP           (1UL << 24)

static const uint8_t kStepPin[NUM_WHEELS] = {
  PIN_STEP_BL, PIN_STEP_BR, PIN_STEP_FL, PIN_STEP_FR
};
static const uint8_t kDirPin[NUM_WHEELS] = {
  PIN_DIR_BL, PIN_DIR_BR, PIN_DIR_FL, PIN_DIR_FR
};
static const int8_t kInvert[NUM_WHEELS] = {
  STEPPER_INVERT_BL ? -1 : 1,
  STEPPER_INVERT_BR ? -1 : 1,
  STEPPER_INVERT_FL ? -1 : 1,
  STEPPER_INVERT_FR ? -1 : 1
};

static volatile int32_t  target_rate[NUM_WHEELS];   // steps/s, signed, wheel frame
static volatile int32_t  current_rate[NUM_WHEELS];  // steps/s, signed, after ramp
static volatile uint32_t phase_inc[NUM_WHEELS];     // Q24 steps per ISR tick
static volatile uint32_t phase_acc[NUM_WHEELS];
static volatile int32_t  step_count[NUM_WHEELS];    // signed microsteps, wheel frame
static volatile int8_t   step_sign[NUM_WHEELS];     // +1 / -1, matches current_rate

static volatile uint16_t status_bits[NUM_WHEELS];
static volatile int16_t  load_result[NUM_WHEELS];

static volatile uint32_t last_command_ms = 0;
static volatile bool     watchdog_tripped = false;
static bool              outputs_enabled = false;

static HardwareTimer StepTimer(TIMER_CH1);

// Steps per second produced by one Dynamixel goal-velocity unit.
static const float kStepsPerDxlUnit =
    DXL_VELOCITY_UNIT_RPS * (float)STEPPER_STEPS_PER_WHEEL_REV;

#if STEPPER_USE_UART
// One-wire UART, one address strap per driver (MS1/MS2 -> 0b00..0b11).
static TMC2209Stepper driverBL(&STEPPER_UART_PORT, STEPPER_RSENSE, 0b00);
static TMC2209Stepper driverBR(&STEPPER_UART_PORT, STEPPER_RSENSE, 0b01);
static TMC2209Stepper driverFL(&STEPPER_UART_PORT, STEPPER_RSENSE, 0b10);
static TMC2209Stepper driverFR(&STEPPER_UART_PORT, STEPPER_RSENSE, 0b11);
static TMC2209Stepper *driver[NUM_WHEELS] = {
  &driverBL, &driverBR, &driverFL, &driverFR
};
#endif

// ---------------------------------------------------------------------------
// Velocity -> step conversion
// ---------------------------------------------------------------------------

float MecanumbotStepper::stepsPerSecondFromDxlUnits(int16_t dxl_units) {
  return (float)dxl_units * kStepsPerDxlUnit;
}

int16_t MecanumbotStepper::dxlUnitsFromStepsPerSecond(float steps_per_second) {
  float units = steps_per_second / kStepsPerDxlUnit;
  if (units >  32767.0f) units =  32767.0f;
  if (units < -32768.0f) units = -32768.0f;
  return (int16_t)lroundf(units);
}

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------

static inline void applyRate(uint8_t i, int32_t rate) {
  current_rate[i] = rate;

  int8_t sign = (rate >= 0) ? 1 : -1;
  step_sign[i] = sign;

  // Physical direction: wheel-frame sign, corrected for mounting and for the
  // driver's DIR polarity.
  bool forward = (sign * kInvert[i]) > 0;
#if !STEPPER_DIR_HIGH_IS_FORWARD
  forward = !forward;
#endif
  digitalWrite(kDirPin[i], forward ? HIGH : LOW);

  uint32_t magnitude = (uint32_t)((rate < 0) ? -rate : rate);
  phase_inc[i] = (uint32_t)(((uint64_t)magnitude << 24) / STEPPER_ISR_HZ);
}

static void stepTimerISR(void) {
  static uint8_t ramp_divider = 0;

  // 1. End any pulse started on the previous tick.
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
    digitalWrite(kStepPin[i], LOW);
  }

  // 2. Ramp toward the target. Steppers stall on an instant speed change, so
  //    this stands in for the XM430's internal profile acceleration.
  if (++ramp_divider >= STEPPER_RAMP_DIVIDER) {
    ramp_divider = 0;
    // Integer ramp step. Guarded so a low STEPPER_ACCEL_STEPS_S2 cannot
    // round down to zero and freeze the ramp entirely.
    const int32_t dv = (STEPPER_ACCEL_STEPS_S2 / STEPPER_RAMP_HZ) > 0
                     ? (STEPPER_ACCEL_STEPS_S2 / STEPPER_RAMP_HZ) : 1;

    for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
      int32_t target = watchdog_tripped ? 0 : target_rate[i];
      int32_t rate   = current_rate[i];

      if (rate < target) {
        rate += dv;
        if (rate > target) rate = target;
      } else if (rate > target) {
        rate -= dv;
        if (rate < target) rate = target;
      } else {
        continue;                       // already there, leave DIR alone
      }
      applyRate(i, rate);
    }
  }

  // 3. Accumulate phase and emit steps.
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
    uint32_t inc = phase_inc[i];
    if (inc == 0) continue;

    uint32_t acc = phase_acc[i] + inc;
    if (acc >= PHASE_ONE_STEP) {
      acc -= PHASE_ONE_STEP;
      step_count[i] += step_sign[i];
      digitalWrite(kStepPin[i], HIGH);
    }
    phase_acc[i] = acc;
  }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void MecanumbotStepper::begin() {
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
    pinMode(kStepPin[i], OUTPUT);
    pinMode(kDirPin[i], OUTPUT);
    digitalWrite(kStepPin[i], LOW);
    digitalWrite(kDirPin[i], LOW);

    target_rate[i]  = 0;
    current_rate[i] = 0;
    phase_inc[i]    = 0;
    phase_acc[i]    = 0;
    step_count[i]   = 0;
    step_sign[i]    = 1;
    status_bits[i]  = STEPPER_ERR_NONE;
    load_result[i]  = 0;
  }

  pinMode(PIN_STEPPER_ENABLE, OUTPUT);
  digitalWrite(PIN_STEPPER_ENABLE, HIGH);   // active LOW: start disabled
  outputs_enabled = false;

#if STEPPER_USE_UART
  STEPPER_UART_PORT.begin(115200);
  delay(50);
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
    driver[i]->begin();
    driver[i]->toff(4);
    driver[i]->rms_current(STEPPER_RUN_CURRENT_MA, STEPPER_HOLD_MULTIPLIER);
    driver[i]->microsteps(STEPPER_MICROSTEPS);
    driver[i]->pwm_autoscale(true);
    driver[i]->en_spreadCycle(false);        // StealthChop: quiet, low speed
    driver[i]->SGTHRS(STEPPER_STALL_THRESHOLD);
    if (driver[i]->test_connection() != 0) {
      status_bits[i] |= STEPPER_ERR_UART_FAIL;
      Serial.print("TMC2209 UART failed on wheel ");
      Serial.println(i);
    }
  }
#endif

  last_command_ms  = millis();
  watchdog_tripped = false;

  StepTimer.stop();
  StepTimer.setPeriod(STEPPER_ISR_PERIOD_US);
  StepTimer.attachInterrupt(stepTimerISR);
  StepTimer.start();

  enable(true);
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

static int32_t clampAndConvert(int16_t dxl_units, uint8_t wheel) {
  int16_t limited = dxl_units;
  if (limited >  STEPPER_MAX_DXL_UNITS) limited =  STEPPER_MAX_DXL_UNITS;
  if (limited < -STEPPER_MAX_DXL_UNITS) limited = -STEPPER_MAX_DXL_UNITS;

  if (limited != dxl_units) {
    status_bits[wheel] |= STEPPER_ERR_CLAMPED;
  } else {
    status_bits[wheel] &= ~STEPPER_ERR_CLAMPED;
  }

  return (int32_t)lroundf(MecanumbotStepper::stepsPerSecondFromDxlUnits(limited));
}

void MecanumbotStepper::setWheelVelocities(int16_t vBL, int16_t vBR,
                                           int16_t vFL, int16_t vFR) {
  target_rate[WHEEL_BL] = clampAndConvert(vBL, WHEEL_BL);
  target_rate[WHEEL_BR] = clampAndConvert(vBR, WHEEL_BR);
  target_rate[WHEEL_FL] = clampAndConvert(vFL, WHEEL_FL);
  target_rate[WHEEL_FR] = clampAndConvert(vFR, WHEEL_FR);
}

void MecanumbotStepper::stopAll() {
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) target_rate[i] = 0;
}

void MecanumbotStepper::enable(bool on) {
  digitalWrite(PIN_STEPPER_ENABLE, on ? LOW : HIGH);   // active LOW
  outputs_enabled = on;
}

bool MecanumbotStepper::isEnabled() {
  return outputs_enabled;
}

void MecanumbotStepper::noteCommandReceived() {
  last_command_ms  = millis();
  watchdog_tripped = false;
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
    status_bits[i] &= ~STEPPER_ERR_CMD_TIMEOUT;
  }
}

void MecanumbotStepper::service() {
  if (!watchdog_tripped &&
      (millis() - last_command_ms) > STEPPER_CMD_TIMEOUT_MS) {
    watchdog_tripped = true;             // the ISR ramps to zero from here
    for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
      status_bits[i] |= STEPPER_ERR_CMD_TIMEOUT;
    }
  }

#if STEPPER_USE_UART
  // Poll one driver per pass: the single-wire UART is slow and the main loop
  // is already carrying Dynamixel and ultrasonic traffic.
  static uint8_t poll_index = 0;
  uint8_t i = poll_index;
  poll_index = (poll_index + 1) % NUM_WHEELS;

  uint32_t drv = driver[i]->DRV_STATUS();
  uint16_t bits = status_bits[i] & (STEPPER_ERR_CMD_TIMEOUT | STEPPER_ERR_CLAMPED);

  if (drv == 0xFFFFFFFFUL) {
    bits |= STEPPER_ERR_UART_FAIL;
  } else {
    if (drv & (1UL << 26)) bits |= STEPPER_ERR_OVERTEMP_WARN;   // otpw
    if (drv & (1UL << 25)) bits |= STEPPER_ERR_OVERTEMP_SHUT;   // ot
    if (drv & (3UL << 27)) bits |= STEPPER_ERR_SHORT;           // s2ga / s2gb
    if (drv & (3UL << 29)) bits |= STEPPER_ERR_OPEN_LOAD;       // ola / olb
    load_result[i] = (int16_t)driver[i]->SG_RESULT();
    // StallGuard is only meaningful once the motor is actually turning.
    if (current_rate[i] != 0 && driver[i]->SG_RESULT() == 0) {
      bits |= STEPPER_ERR_STALL;
    }
  }
  status_bits[i] = bits;
#endif
}

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

void MecanumbotStepper::getMeasuredVelocities(int16_t out[NUM_WHEELS]) {
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
    out[i] = dxlUnitsFromStepsPerSecond((float)current_rate[i]);
  }
}

void MecanumbotStepper::getStepCounts(int32_t out[NUM_WHEELS]) {
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) out[i] = step_count[i];
}

void MecanumbotStepper::getPositionTicks(int16_t out[NUM_WHEELS]) {
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) {
    // Same 4096-ticks-per-revolution scale the XM430 reported, so host-side
    // maths is unchanged. It wraps in int16 exactly as the old field did.
    int32_t ticks = (int32_t)(((int64_t)step_count[i] * 4096) /
                              STEPPER_STEPS_PER_WHEEL_REV);
    out[i] = (int16_t)ticks;
  }
}

void MecanumbotStepper::getAccelUnits(int16_t out[NUM_WHEELS]) {
  int16_t units = dxlUnitsFromStepsPerSecond((float)STEPPER_ACCEL_STEPS_S2);
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) out[i] = units;
}

void MecanumbotStepper::getDriverStatus(int16_t out[NUM_WHEELS]) {
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) out[i] = (int16_t)status_bits[i];
}

void MecanumbotStepper::getLoadResult(int16_t out[NUM_WHEELS]) {
  for (uint8_t i = 0; i < NUM_WHEELS; ++i) out[i] = load_result[i];
}
