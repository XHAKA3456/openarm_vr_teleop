#!/usr/bin/env python3
"""
Sync neck motors with Meta Quest head tracking.
- head.euler.x (+위로 까딱): Controls nod (ID 6) - up/down nod
- head.euler.y (+좌로 회전): Controls rotate (ID 5) - left/right rotation
- head.euler.z (+우로 까딱): Controls tilt (ID 4) - left/right tilt

Press Ctrl+C to quit.
"""

import json
import socket
import sys
import time
from controller import STS3215Controller

# Network settings
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5454

# Initial motor positions (measured from home position)
INITIAL_NOD = 2722     # Up/Down nod (ID 6)
INITIAL_ROTATE = 3096  # Left/Right rotation (ID 5)
INITIAL_TILT = 1243    # Left/Right tilt (ID 4)

# Position limits
MIN_POS = 0
MAX_POS = 4095

# Scaling factors (adjust these to tune sensitivity)
# Degrees to raw ticks conversion
SCALE_YAW = 15.0    # For left/right rotation (rotate, ID 5)
SCALE_PITCH = 15.0  # For up/down nod (nod, ID 6)
SCALE_ROLL = 15.0   # For left/right tilt (tilt, ID 4)

# Optional: limit the euler angle range to prevent extreme positions
MAX_ANGLE_DELTA = 45.0  # degrees


def clamp(value, min_val, max_val):
    """Clamp value between min and max."""
    return max(min_val, min(max_val, value))


def normalize_angle_delta(delta):
    """Normalize angle delta to [-180, 180] range."""
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    return delta


def parse_quest_data(line):
    """Parse JSON data from Meta Quest and extract head euler angles."""
    try:
        # Find JSON part (starts with '{')
        json_start = line.find('{')
        if json_start == -1:
            return None

        data = json.loads(line[json_start:])

        if 'head' in data and 'euler' in data['head']:
            euler = data['head']['euler']
            return {
                'x': euler['x'],  # pitch (up/down nod)
                'y': euler['y'],  # yaw (left/right rotation)
                'z': euler['z']   # roll (left/right tilt)
            }
    except (json.JSONDecodeError, KeyError) as e:
        print(f"Parse error: {e}", file=sys.stderr)
    return None


def main():
    # Setup TCP socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((UDP_IP, UDP_PORT))
    server.listen(1)

    print(f"Listening for Meta Quest data on TCP port {UDP_PORT}...")
    print("Waiting for a connection...")

    conn, addr = server.accept()
    print(f"Connected: {addr}")
    print()

    controller = STS3215Controller(port="/dev/ttyUSB0")
    controller.open()
    controller.enable_torque(True)

    print("Meta Quest Head Tracking Sync")
    print("=" * 50)
    print(f"Initial motor positions:")
    print(f"  nod (ID 6):    {INITIAL_NOD} (Up/Down nod)")
    print(f"  rotate (ID 5): {INITIAL_ROTATE} (Left/Right rotation)")
    print(f"  tilt (ID 4):   {INITIAL_TILT} (Left/Right tilt)")
    print()
    print("Waiting for calibration...")
    print("Hold your head in the neutral position and the system will auto-calibrate in 3 seconds.")
    print()

    # Move to initial positions
    controller.command_positions({
        "nod": INITIAL_NOD,
        "rotate": INITIAL_ROTATE,
        "tilt": INITIAL_TILT
    })

    # Calibration: read first few samples and average
    calibration_samples = []
    calibration_start = None
    calibrated = False
    initial_yaw = 0.0
    initial_pitch = 0.0
    initial_roll = 0.0

    try:
        with conn:
            buffer = ""
            while True:
                try:
                    data = conn.recv(1024)
                    if not data:
                        print("Connection closed by client")
                        break

                    # Decode and split by lines
                    buffer += data.decode('utf-8', errors='ignore')
                    lines = buffer.split('\n')
                    buffer = lines[-1]  # Keep incomplete line in buffer

                    for line in lines[:-1]:
                        if not line.strip():
                            continue

                        euler = parse_quest_data(line)
                        if euler is None:
                            continue

                        # Calibration phase (first 3 seconds)
                        if not calibrated:
                            # Start calibration timer on first valid data
                            if calibration_start is None:
                                calibration_start = time.time()
                                print("First data received! Starting calibration...")
                                print("Hold your head in neutral position for 3 seconds...")

                            calibration_samples.append(euler)
                            elapsed = time.time() - calibration_start

                            if elapsed >= 3.0:
                                # Calculate average initial position
                                initial_yaw = sum(s['y'] for s in calibration_samples) / len(calibration_samples)
                                initial_pitch = sum(s['x'] for s in calibration_samples) / len(calibration_samples)
                                initial_roll = sum(s['z'] for s in calibration_samples) / len(calibration_samples)

                                print(f"✓ Calibration complete!")
                                print(f"  Initial yaw (y):   {initial_yaw:.2f}° (좌우 회전)")
                                print(f"  Initial pitch (x): {initial_pitch:.2f}° (위아래 까딱)")
                                print(f"  Initial roll (z):  {initial_roll:.2f}° (좌우 까딱)")
                                print()
                                print("Now tracking head movements...")
                                print("-" * 50)
                                calibrated = True
                            continue

                        # Calculate deltas from initial position
                        current_yaw = euler['y']
                        current_pitch = euler['x']
                        current_roll = euler['z']

                        delta_yaw = normalize_angle_delta(current_yaw - initial_yaw)
                        delta_pitch = normalize_angle_delta(current_pitch - initial_pitch)
                        delta_roll = normalize_angle_delta(current_roll - initial_roll)

                        # Clamp angle deltas to safe range
                        delta_yaw = clamp(delta_yaw, -MAX_ANGLE_DELTA, MAX_ANGLE_DELTA)
                        delta_pitch = clamp(delta_pitch, -MAX_ANGLE_DELTA, MAX_ANGLE_DELTA)
                        delta_roll = clamp(delta_roll, -MAX_ANGLE_DELTA, MAX_ANGLE_DELTA)

                        # Convert angle deltas to motor positions
                        # Yaw (좌우 회전): +y → rotate (ID 5)
                        rotate_pos = INITIAL_ROTATE + int(delta_yaw * SCALE_YAW)

                        # Pitch (위아래 까딱): +x → nod (ID 6)
                        nod_pos = INITIAL_NOD + int(delta_pitch * SCALE_PITCH)

                        # Roll (좌우 까딱): +z → tilt (ID 4)
                        tilt_pos = INITIAL_TILT - int(delta_roll * SCALE_ROLL)

                        # Clamp to valid position range
                        rotate_pos = clamp(rotate_pos, MIN_POS, MAX_POS)
                        nod_pos = clamp(nod_pos, MIN_POS, MAX_POS)
                        tilt_pos = clamp(tilt_pos, MIN_POS, MAX_POS)

                        # Send commands
                        controller.command_positions({
                            "nod": nod_pos,
                            "rotate": rotate_pos,
                            "tilt": tilt_pos
                        })

                        # Print status (commented out for cleaner output)
                        # print(f"Yaw: {delta_yaw:+6.1f}° → rotate: {rotate_pos:4d} | "
                        #       f"Pitch: {delta_pitch:+6.1f}° → nod: {nod_pos:4d} | "
                        #       f"Roll: {delta_roll:+6.1f}° → tilt: {tilt_pos:4d}", end='\r')

                except Exception as e:
                    print(f"\nSocket error: {e}", file=sys.stderr)
                    break

    except KeyboardInterrupt:
        print("\n\nStopping...")
    except ConnectionResetError:
        print("\nConnection lost")
    finally:
        print("\nDisabling torque and closing...")
        controller.enable_torque(False)
        controller.close()
        server.close()


if __name__ == "__main__":
    main()
