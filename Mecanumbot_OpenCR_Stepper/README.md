# Mecanumbot OpenCR — NEMA17 / TMC2209 wheel drive

Drop-in replacement for `Mecanumbot_OpenCR` in which the four **XM430-W210-T**
wheel Dynamixels are replaced by **NEMA17 steppers driven by TMC2209** modules
in STEP/DIR mode.

The neck and grabber **AX-12A motors are unchanged** and still run on the
Dynamixel bus. The host wire protocol is **byte-for-byte identical**, so
`test_cr.py` and the ROS 2 stack need no changes.

> Status: written against the OpenCR 1.5.3 core, but **never run on hardware**.
> The step-generation logic (unit conversion, ramp, pulse rate, direction,
> counters, clamping, watchdog) was verified on a host build; the wiring,
> currents and mechanical fit have not been. Read *Bring-up* before powering
> motors.

## Is the swap feasible?

Yes, electrically and computationally. The real costs are feedback and weight.

**Speed and step rate.** The Dynamixel build commands at most ~200 velocity
units = 45.8 rev/min = **0.31 m/s** at the 6.5 cm wheel. At 1/16 microstepping
that is 200 × 16 = 3200 steps/rev, so:

```
2442 steps/s per wheel   (~9.8 kHz for all four)
```

The 20 kHz ISR here tops out at 20000 steps/s per wheel (375 rev/min), so the
robot's working range uses about 12% of the available rate. On the STM32F746 at
216 MHz the ISR costs roughly 3% CPU.

**Torque.** Rolling plus 1 m/s² acceleration on a ~6 kg robot needs on the
order of 0.15 N·m per wheel; a 42×48 mm NEMA17 holds ~0.45 N·m. At 45 rev/min
the motor is far below its torque roll-off, so the margin is comfortable — a
stepper's weakness is high speed, and this robot is slow.

**What you lose.** This is the part that matters for the research, not the
electronics:

- **No feedback.** The XM430 reported present velocity, position, current and
  a hardware error status. A TMC2209 in STEP/DIR mode reports nothing. Wheel
  telemetry below is *commanded*, not measured — a skipped step or a stalled
  wheel is silent and shows up only as odometry drift. Enable the UART option
  for StallGuard if you need stall detection, or add encoders if you need real
  odometry.
- **Weight.** 4 × NEMA17 ≈ 1.14 kg against 4 × XM430 ≈ 0.33 kg, all of it
  unsprung at the corners, plus new mounts and shaft couplings for the mecanum
  hubs.
- **Standstill current.** Steppers draw holding current while stopped. Use the
  TMC2209 hold-current reduction (UART mode sets it via `IHOLD`) or drop `/EN`
  when parked — but note that dropping `/EN` also lets the robot roll freely.
- **Power.** The drivers must be fed from the battery through their own fuse,
  **not** from OpenCR's 12 V rail, which cannot supply four motors. TMC2209
  takes 4.75–29 V. At the 11.1 V TurtleBot3 LiPo the motors work but have less
  headroom; 24 V would be better if the rest of the robot tolerates it.

## Wiring

Shared `/EN`, one STEP and one DIR per wheel. GPIO 5, 9, 13, 17 stay reserved
for the oscilloscope timing rig and Arduino pins 2/3 for the ultrasonic sensor.

| Signal | OpenCR pin | | Signal | OpenCR pin |
| --- | --- | --- | --- | --- |
| STEP BL | `BDPIN_GPIO_1` | | DIR BL | `BDPIN_GPIO_2` |
| STEP BR | `BDPIN_GPIO_3` | | DIR BR | `BDPIN_GPIO_4` |
| STEP FL | `BDPIN_GPIO_6` | | DIR FL | `BDPIN_GPIO_7` |
| STEP FR | `BDPIN_GPIO_8` | | DIR FR | `BDPIN_GPIO_10` |
| `/EN` (all four, active LOW) | `BDPIN_GPIO_11` | | | |

Grounds must be common between OpenCR and the driver modules. In standalone
mode set microstepping with the MS1/MS2 straps to match
`STEPPER_MICROSTEPS`, and set the run current with the VREF pot **before**
the first move.

## Configuration

Everything tunable is at the top of `mecanumbot_stepper.h`:

| Define | Meaning |
| --- | --- |
| `STEPPER_MICROSTEPS` | must match the MS1/MS2 straps (or the UART setting) |
| `STEPPER_GEAR_RATIO` | motor revolutions per wheel revolution, 1.0 for direct drive |
| `STEPPER_INVERT_*` | per-wheel direction sign — flip here, not in the wiring |
| `STEPPER_MAX_DXL_UNITS` | command clamp, 330 units ≈ 75 rev/min |
| `STEPPER_ACCEL_STEPS_S2` | ramp rate; replaces the XM430 profile acceleration |
| `STEPPER_CMD_TIMEOUT_MS` | wheels ramp to zero if the host goes quiet |
| `STEPPER_USE_UART` | 1 to configure current/microsteps/StallGuard over UART |

`STEPPER_USE_UART` needs the **TMCStepper** library and the four MS1/MS2
address straps (0b00…0b11); it is off by default so the sketch builds with only
the OpenCR core and DynamixelSDK.

## How the step generator works

`MecanumbotStepper::stepsPerSecondFromDxlUnits()` is the conversion the module
is built around. The host keeps sending Dynamixel goal-velocity units
(1 unit = 0.229 rev/min), and:

```
steps/s = units × (0.229 / 60) × full_steps × microsteps × gear_ratio
```

A 20 kHz hardware-timer ISR (`TIMER_CH1`; `TIMER_TONE` is left alone for the
buzzer) runs a Q24 phase accumulator per wheel and emits one STEP pulse per
overflow. Rate-to-increment conversion happens on the 1 kHz ramp tick, so the
fast path is an add, a compare and a `digitalWrite`.

Running the pulse train from an ISR is what makes the swap work at all: the
main loop still blocks on Dynamixel writes and on `pulseIn()` for the
ultrasonic sensor, and stepping continues through all of it.

## Telemetry field changes

The packet layout is unchanged (`<27h15f`, 114-byte payload, 118-byte frame,
magic `0xAA55`, CRC8-CCITT). Four groups of wheel fields change meaning
because the underlying sensor is gone:

| Field | Dynamixel build | This build |
| --- | --- | --- |
| `vel_*` | measured present velocity | **commanded** ramped rate, converted back to Dynamixel units |
| `pos_*` | measured present position | integrated step count, same 4096-ticks-per-rev scale |
| `curr_*` | present current | StallGuard load in UART mode, otherwise `0` |
| `acc_*` | profile acceleration | configured ramp rate, in Dynamixel units/s |
| `err_*` | Dynamixel hardware error | `STEPPER_ERR_*` bits (stall, over-temp, short, open load, command timeout, clamped, UART fail) |

Anything consuming `vel_*`/`pos_*` as ground truth — odometry above all — is
now reading an open-loop estimate. That is the one change the ROS 2 side has to
account for.

## Behaviour changes from the Dynamixel build

- **Command watchdog.** No packet for `STEPPER_CMD_TIMEOUT_MS` ramps the wheels
  to a stop. The old firmware tracked `last_command_time` but never acted on
  it, so a lost link left the last velocity latched — worse with steppers.
- **`delay(100)` removed.** The old command path blocked for 100 ms to hold the
  scope-rig pins high. The pins are now pulsed for `DIAG_PULSE_MS` non-blocking
  (the logger in `oscilloscope/` polls every ~4 µs, so it still catches them).
- **Explicit telemetry rate.** The old loop was throttled incidentally by two
  blocking wheel sync-reads. Those are gone, so `TELEMETRY_HZ` (100 Hz) now
  sets the cadence rather than letting the loop flood the link.
- **Single Dynamixel protocol.** With the XM430s gone the bus is protocol 1
  only; no more protocol switching around wheel writes.

## Bring-up

Do this on blocks, wheels off the ground.

1. Set VREF on each driver for your motor's rated current before connecting
   motor coils. Start low.
2. Flash, and confirm telemetry arrives with `test_cr.py` (wheels commanded to
   zero). Before the first command `err_*` reads `0x20`
   (`STEPPER_ERR_CMD_TIMEOUT`) — that is correct, and clears on the first
   packet. Any other bit set is a real fault.
3. Command one wheel at a time at ~50 units. Check direction, and fix any wrong
   one with `STEPPER_INVERT_*` — not by re-wiring.
4. Verify the pulse rate: at 200 units a wheel should turn 45.8 rev/min.
   If it is out by an integer factor, `STEPPER_MICROSTEPS` disagrees with the
   MS1/MS2 straps.
5. Raise the current until the wheels no longer skip steps under load, then
   check driver temperature after a few minutes of driving.
6. Only then re-tune odometry on the ROS 2 side.

## Known limitations

- The downlink has no framing: it is a bare 14-byte struct, so one lost or
  spurious byte desynchronizes the stream until the host reconnects. This is
  inherited unchanged from the Dynamixel build, deliberately, to keep the
  protocol identical — but it is worth fixing on both sides eventually.
- At the 20 kHz ISR ceiling the STEP low time shrinks to a few hundred ns.
  Well outside the robot's working range, but do not raise
  `STEPPER_MAX_DXL_UNITS` toward the ceiling without checking the pulse shape.
- Step counts are commanded, not measured — see above.
