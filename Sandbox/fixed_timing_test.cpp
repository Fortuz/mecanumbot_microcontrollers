// Minimal Arduino-friendly timing skeleton with DYNAMIXEL-friendly cadence.
#include <Arduino.h>
// #include <Dynamixel2Arduino.h> // or your motor lib

// Timing periods (microseconds)
static const uint32_t TOGGLE_PERIOD_US = 500000;  // 500 ms
static const uint32_t CTRL_PERIOD_US   = 10000;   // 10 ms (100 Hz control loop)

static uint32_t next_toggle_us;
static uint32_t next_ctrl_us;
static bool switcher_ax_motors = false;

// Wraparound-safe "now >= deadline" check
inline bool time_due(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

void setup() {
  // Serial.begin(115200); // Be careful with prints in the timing path
  // dxl.begin(1000000);   // e.g., 1 Mbps, depends on your setup
  // dxl.setPortProtocolVersion(2.0);

  uint32_t now = micros();
  next_toggle_us = now + TOGGLE_PERIOD_US;
  next_ctrl_us   = now + CTRL_PERIOD_US;

  // Serial.println("Starting fixed timing test...");
}

void control_step() {
  // 1) Compute commands
  // 2) Batch motor writes: prefer SyncWrite/BulkWrite once per control tick
  // Example (pseudo):
  // syncWriteVelocity(ids, velocities, count);
  // syncWritePosition(ids, positions, count);
  // Avoid txRx in this hot loop unless needed; keep it write-mostly.
}

void slow_telemetry_step() {
  // Optional: do slower reads (voltage/temperature) at lower rate
  // e.g., every N control ticks, use BulkRead/Indirect to minimize traffic
}

void loop() {
  uint32_t now = micros();

  // Control loop (e.g., 100 Hz)
  if (time_due(now, next_ctrl_us)) {
    // Catch up if we slipped
    do {
      next_ctrl_us += CTRL_PERIOD_US;
    } while (time_due(now, next_ctrl_us));

    control_step();

    // Optionally do slower telemetry every, say, 50 ms:
    // static uint8_t ctr = 0;
    // if ((++ctr % 5) == 0) slow_telemetry_step();
  }

  // 500 ms toggle
  if (time_due(now, next_toggle_us)) {
    do {
      next_toggle_us += TOGGLE_PERIOD_US;
    } while (time_due(now, next_toggle_us));

    switcher_ax_motors = !switcher_ax_motors;
    // Avoid Serial prints here in production; they add jitter
    // Serial.println(switcher_ax_motors ? "Advanced motors..." : "Regular motors...");
  }

  // Keep loop responsive; avoid long blocking calls
  // yield(); // OK
  // delayMicroseconds(50); // tiny sleep if needed, but keep it small
}