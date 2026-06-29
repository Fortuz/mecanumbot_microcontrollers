import serial
import struct
import time
import threading
import sys
from pynput import keyboard

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0' 
BAUDRATE = 1000000

# --- Mecanumbot Constants ---
MECANUMBOT_MIN_CAM_POS = 2.0
MECANUMBOT_MID_CAM_POS = 5.12
MECANUMBOT_MAX_CAM_POS = 8.6

MECANUMBOT_MIN_GRIPPER_POS = 1.6
MECANUMBOT_FRONT_GRIPPER_POS = 5.12
MECANUMBOT_MAX_GRIPPER_POS = 8.54

# --- Global Control State ---
control_state = {
    "vel_BL": 0, "vel_BR": 0, "vel_FL": 0, "vel_FR": 0,
    "pos_N": MECANUMBOT_MID_CAM_POS,
    "pos_GL": MECANUMBOT_FRONT_GRIPPER_POS,
    "pos_GR": MECANUMBOT_FRONT_GRIPPER_POS
}
running = True
state_changed = True  # Starts as True to send the initial boot packet ONCE

# --- CRC8-CCITT (poly 0x07) ---
def crc8_ccitt(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

# --- Keyboard Callbacks ---
def on_press(key):
    global control_state, state_changed
    try:
        if key.char == 'i':
            control_state["pos_N"] = MECANUMBOT_MAX_CAM_POS
            state_changed = True
        elif key.char == 'k':
            control_state["pos_N"] = MECANUMBOT_MIN_CAM_POS
            state_changed = True
        elif key.char == 'j':
            control_state["pos_GL"] = (MECANUMBOT_MAX_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS) / 2.0
            control_state["pos_GR"] = (MECANUMBOT_MIN_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS) / 2.0
            state_changed = True
        elif key.char == 'l':
            control_state["pos_GL"] = (MECANUMBOT_MIN_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS) / 2.0
            control_state["pos_GR"] = (MECANUMBOT_MAX_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS) / 2.0
            state_changed = True
    except AttributeError:
        pass 

def on_release(key):
    global running
    if key == keyboard.Key.esc:
        running = False
        return False

# --- Labels for Unpacking ---
SENSOR_LABELS = [
    "cmd_vel_BL", "cmd_vel_BR", "cmd_vel_FL", "cmd_vel_FR",
    "vel_BL", "vel_BR", "vel_FL", "vel_FR",
    "pos_BL", "pos_BR", "pos_FL", "pos_FR",
    "curr_BL", "curr_BR", "curr_FL", "curr_FR",
    "acc_BL", "acc_BR", "acc_FL", "acc_FR",
    "pos_N", "pos_GL", "pos_GR",
    "err_BL", "err_BR", "err_FL", "err_FR",
    "dms_distance", "voltage",
    "imu_ang_vel_x", "imu_ang_vel_y", "imu_ang_vel_z",
    "imu_lin_acc_x", "imu_lin_acc_y", "imu_lin_acc_z",
    "imu_mag_x", "imu_mag_y", "imu_mag_z",
    "orientation_w", "orientation_x", "orientation_y", "orientation_z"
]

# --- Serial Read Thread ---
def serial_read_loop(ser):
    payload_format = '<27h15f'
    
    while running:
        if ser.in_waiting >= 118:
            if ser.read(1) == b'\x55' and ser.read(1) == b'\xaa':
                packet_rest = ser.read(116)
                if len(packet_rest) == 116:
                    seq = packet_rest[0]
                    payload_data = packet_rest[1:115]
                    recv_crc = packet_rest[115]
                    
                    calc_crc = crc8_ccitt(b'\x55\xaa' + packet_rest[:115])
                    
                    if calc_crc == recv_crc:
                        unpacked = struct.unpack(payload_format, payload_data)
                        
                        sys.stdout.write("\033[H\033[J")
                        sys.stdout.write(f"=== TELEMETRY PACKET | SEQ: {seq:3d} ===\n")
                        
                        for label, val in zip(SENSOR_LABELS, unpacked):
                            if isinstance(val, float):
                                sys.stdout.write(f"{label:20}: {val:10.4f}\n")
                            else:
                                sys.stdout.write(f"{label:20}: {val:10d}\n")
                                
                        sys.stdout.write("======================================\n")
                        sys.stdout.flush()

# --- Main Application ---
def main():
    global state_changed
    
    print(f"Connecting to OpenCR on {SERIAL_PORT} @ {BAUDRATE} bps...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    print("Connected! Starting telemetry stream...")
    time.sleep(2) 

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    read_thread = threading.Thread(target=serial_read_loop, args=(ser,), daemon=True)
    read_thread.start()

    control_format = '<7h' 
    
    try:
        while running:
            # ONLY send data if a key was pressed (or on first boot)
            if state_changed:
                buf = struct.pack(control_format,
                    control_state["vel_BL"],
                    control_state["vel_BR"],
                    control_state["vel_FL"],
                    control_state["vel_FR"],
                    int(control_state["pos_N"] * 100),
                    int(control_state["pos_GL"] * 100),
                    int(control_state["pos_GR"] * 100)
                )
                ser.write(buf)
                
                # Reset the flag so we don't send again until the next keypress
                state_changed = False 
                
            time.sleep(0.05) # Keep the main thread alive without blocking the CPU
            
    except KeyboardInterrupt:
        pass
    finally:
        print("\nClosing connection...")
        listener.stop()
        ser.close()

if __name__ == '__main__':
    main()