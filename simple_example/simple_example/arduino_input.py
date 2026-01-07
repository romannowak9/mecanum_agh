import serial
import time
import struct

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'  # Change to 'COM3' on Windows
BAUD_RATE = 9600

# Calibration (Multipliers to adjust drift)
CALIB_L = 1.0 
CALIB_R = 1.0

def send_message(arduino, command, velocity, distance=0, mode=250):
    """
    Sends data matching the Arduino 'dataBuffer' logic.
    
    :param arduino: Serial object
    :param command: 'go', 'back', 'left', 'right', 'stop'
    :param velocity: 0-255
    :param distance: 0-255 (Arduino multiplies this by 10 for encoder limit)
    :param mode: 
        250 = Advanced Control (Stop after distance)
        240 = Continuous Run (Ignore distance)
        255 = Emergency Stop
    """
    
    # Calculate calibrated speeds
    speed_l = int(velocity * CALIB_L)
    speed_r = int(velocity * CALIB_R)
    
    # Constrain to byte size (0-255)
    speed_l = max(0, min(255, speed_l))
    speed_r = max(0, min(255, speed_r))
    
    # Default speeds for 4 motors (TL, TR, BL, BR)
    sTL, sTR, sBL, sBR = speed_l, speed_r, speed_l, speed_r
    
    # Default directions (TL, TR, BL, BR)
    dTL, dTR, dBL, dBR = 0, 0, 0, 0

    # --- DIRECTION LOGIC (Matched to your previous views.py) ---
    if command == 'go':
        # Pattern: 1, 0, 1, 0
        dTL, dTR, dBL, dBR = 1, 0, 1, 0
    elif command == 'back':
        # Pattern: 0, 1, 0, 1
        dTL, dTR, dBL, dBR = 0, 1, 0, 1
    elif command == 'left':
        # Pattern: 0, 0, 0, 0
        dTL, dTR, dBL, dBR = 0, 0, 0, 0
    elif command == 'right':
        # Pattern: 1, 1, 1, 1
        dTL, dTR, dBL, dBR = 1, 1, 1, 1
    elif command == 'left_rotate':
         # Pattern from views.py cmd 5
        dTL, dTR, dBL, dBR = 0, 0, 1, 1
    elif command == 'right_rotate':
        # Pattern from views.py cmd 6
        dTL, dTR, dBL, dBR = 1, 1, 0, 0
    elif command == 'stop':
        mode = 255
        distance = 0

    # --- PACKET CONSTRUCTION ---
    packet = []
    
    if mode == 255:
        # Arduino: if 255, expectedDataSize = 1
        packet = [255, 0] 
    else:
        # Arduino: if 240/250, expectedDataSize = 9
        # Structure: [Header, Dist, S_TL, S_TR, S_BL, S_BR, D_TL, D_TR, D_BL, D_BR]
        packet = [
            mode,       # Header
            distance,   # dataBuffer[0]
            sTL,        # dataBuffer[1]
            sTR,        # dataBuffer[2]
            sBL,        # dataBuffer[3]
            sBR,        # dataBuffer[4]
            dTL,        # dataBuffer[5]
            dTR,        # dataBuffer[6]
            dBL,        # dataBuffer[7]
            dBR         # dataBuffer[8]
        ]

    # --- SENDING ---
    try:
        print(f"Sending: {packet}")
        arduino.write(bytes(packet))
        
        # If using Advanced Control (250), Arduino sends back "true" when done
        if mode == 250:
            print("Waiting for movement to finish...")
            while True:
                if arduino.in_waiting > 0:
                    response = arduino.readline().decode('utf-8').strip()
                    print(f"Arduino: {response}")
                    if response == "true":
                        break
    except Exception as e:
        print(f"Transmission Error: {e}")

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    try:
        arduino = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1)
        time.sleep(2) # Allow Arduino to reset
        print("Connected.")

        # --- EXAMPLE 1: Move Forward for a specific distance (Mode 250) ---
        # Distance 20 sends "20" to Arduino, which calculates "200" ticks
        #send_message(arduino, command='go', velocity=150, distance=20, mode=250)
        
        time.sleep(1)

        # --- EXAMPLE 2: Move Backwards continuously (Mode 240) ---
        send_message(arduino, command='back', velocity=120, distance=0, mode=240)
        
        time.sleep(2) # Let it run for 2 seconds

        # --- EXAMPLE 3: Stop ---
        send_message(arduino, command='stop', velocity=0, mode=255)

        arduino.close()
        print("Done.")

    except serial.SerialException:
        print(f"Could not open {SERIAL_PORT}")