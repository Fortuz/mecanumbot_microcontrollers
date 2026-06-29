import serial
import time

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB0'  # Change this to /dev/arduino_nano if udev is working
BAUD_RATE = 115200

START_BYTE = 0xAA
REQUEST_BYTE = 0xAC

def build_packet(fl_m, fl_c, fr_m, fr_c, bl_m, bl_c, br_m, br_c, duration_ms):
    """Builds the 12-byte packet exactly as your Arduino expects it."""
    tL = duration_ms & 0xFF
    tH = (duration_ms >> 8) & 0xFF
    # XOR checksum based on your Arduino logic
    checksum = fl_m ^ fl_c ^ fr_m ^ fr_c ^ bl_m ^ bl_c ^ br_m ^ br_c ^ tL ^ tH
    
    return bytes([
        START_BYTE, 
        fl_m, fl_c, 
        fr_m, fr_c, 
        bl_m, bl_c, 
        br_m, br_c, 
        tL, tH, 
        checksum
    ])

def main():
    print(f"Opening {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except Exception as e:
        print(f"CRITICAL ERROR: Failed to open port. \n{e}")
        return

    # CRITICAL: Wait for the Arduino to get past the delay(3000) in setup()
    print("Waiting 4 seconds for Arduino to initialize...")
    time.sleep(4.0)

    # ==========================================
    # TEST 1: Request feedback from Arduino
    # ==========================================
    print("\n--- TEST 1: Requesting Status ---")
    ser.reset_input_buffer() # Clear out any junk data
    ser.write(bytes([REQUEST_BYTE]))
    
    # Wait a tiny bit for the Arduino to process and reply
    time.sleep(0.05) 
    
    if ser.in_waiting > 0:
        reply = ser.read(ser.in_waiting)
        formatted_reply = [hex(b) for b in reply]
        print(f"SUCCESS: Received {len(reply)} bytes: {formatted_reply}")
        if reply[0] == 0xAB:
            print("Feedback start byte (0xAB) verified!")
    else:
        print("FAIL: No response received from Arduino. Check wiring/port.")

    # ==========================================
    # TEST 2: Send LED Command
    # ==========================================
    print("\n--- TEST 2: Sending LED Command ---")
    # Using some arbitrary values (e.g., Mode 1, Color 2). 
    # Change these if you know specific mode/color INTs from your 'LEDutils.h'
    packet = build_packet(
        fl_m=1, fl_c=2, 
        fr_m=1, fr_c=2, 
        bl_m=1, bl_c=2, 
        br_m=1, br_c=2, 
        duration_ms=1000
    )
    
    ser.write(packet)
    print(f"Packet sent: {[hex(b) for b in packet]}")
    print("Check your LEDs. Did they change?")
     # ==========================================
    # TEST 1: Request feedback from Arduino
    # ==========================================
    print("\n--- TEST 3: Requesting Status ---")
    ser.reset_input_buffer() # Clear out any junk data
    ser.write(bytes([REQUEST_BYTE]))
    
    # Wait a tiny bit for the Arduino to process and reply
    time.sleep(0.05) 
    
    if ser.in_waiting > 0:
        reply = ser.read(ser.in_waiting)
        formatted_reply = [hex(b) for b in reply]
        print(f"SUCCESS: Received {len(reply)} bytes: {formatted_reply}")
        if reply[0] == 0xAB:
            print("Feedback start byte (0xAB) verified!")
    else:
        print("FAIL: No response received from Arduino. Check wiring/port.")

    ser.close()
    print("\nTest complete. Port closed.")


if __name__ == '__main__':
    main()