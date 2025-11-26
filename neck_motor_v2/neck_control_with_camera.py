#!/usr/bin/env python3
"""
Integrated neck control with camera viewer.
- Receives Meta Quest head tracking data and controls neck motors
- Displays USB camera feed on screen
"""

import cv2
import json
import socket
import sys
import time
import threading
from controller import STS3215Controller

# Network settings
UDP_IP = "0.0.0.0"
UDP_PORT = 5454

# Camera settings
CAMERA_INDEX = 2
WINDOW_NAME = "Neck Camera"
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Initial motor positions (measured from home position)
INITIAL_NOD = 2722     # Up/Down nod (ID 6)
INITIAL_ROTATE = 3096  # Left/Right rotation (ID 5)
INITIAL_TILT = 1243    # Left/Right tilt (ID 4)

# Position limits
MIN_POS = 0
MAX_POS = 4095

# Scaling factors
SCALE_YAW = 15.0    # For left/right rotation (rotate, ID 5)
SCALE_PITCH = 15.0  # For up/down nod (nod, ID 6)
SCALE_ROLL = 15.0   # For left/right tilt (tilt, ID 4)
MAX_ANGLE_DELTA = 45.0


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


def motor_control_thread(controller, stop_event):
    """Thread for receiving Quest data and controlling motors."""
    # Setup TCP socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((UDP_IP, UDP_PORT))
    server.listen(1)

    print(f"[Motor Control] Listening on TCP port {UDP_PORT}...")
    print("[Motor Control] Waiting for Meta Quest connection...")

    conn, addr = server.accept()
    print(f"[Motor Control] Connected: {addr}")

    # Move to initial positions
    controller.command_positions({
        "nod": INITIAL_NOD,
        "rotate": INITIAL_ROTATE,
        "tilt": INITIAL_TILT
    })

    # Calibration variables
    calibration_samples = []
    calibration_start = None
    calibrated = False
    initial_yaw = 0.0
    initial_pitch = 0.0
    initial_roll = 0.0

    try:
        with conn:
            buffer = ""
            while not stop_event.is_set():
                try:
                    data = conn.recv(1024)
                    if not data:
                        print("[Motor Control] Connection closed")
                        break

                    buffer += data.decode('utf-8', errors='ignore')
                    lines = buffer.split('\n')
                    buffer = lines[-1]

                    for line in lines[:-1]:
                        if not line.strip():
                            continue

                        euler = parse_quest_data(line)
                        if euler is None:
                            continue

                        # Calibration phase
                        if not calibrated:
                            if calibration_start is None:
                                calibration_start = time.time()
                                print("[Motor Control] First data received! Starting calibration...")
                                print("[Motor Control] Hold head in neutral position for 3 seconds...")

                            calibration_samples.append(euler)
                            elapsed = time.time() - calibration_start

                            if elapsed >= 3.0:
                                initial_yaw = sum(s['y'] for s in calibration_samples) / len(calibration_samples)
                                initial_pitch = sum(s['x'] for s in calibration_samples) / len(calibration_samples)
                                initial_roll = sum(s['z'] for s in calibration_samples) / len(calibration_samples)

                                print(f"[Motor Control] ✓ Calibration complete!")
                                print(f"[Motor Control]   Initial yaw (y):   {initial_yaw:.2f}°")
                                print(f"[Motor Control]   Initial pitch (x): {initial_pitch:.2f}°")
                                print(f"[Motor Control]   Initial roll (z):  {initial_roll:.2f}°")
                                print("[Motor Control] Now tracking head movements...")
                                calibrated = True
                            continue

                        # Calculate deltas
                        current_yaw = euler['y']
                        current_pitch = euler['x']
                        current_roll = euler['z']

                        delta_yaw = normalize_angle_delta(current_yaw - initial_yaw)
                        delta_pitch = normalize_angle_delta(current_pitch - initial_pitch)
                        delta_roll = normalize_angle_delta(current_roll - initial_roll)

                        delta_yaw = clamp(delta_yaw, -MAX_ANGLE_DELTA, MAX_ANGLE_DELTA)
                        delta_pitch = clamp(delta_pitch, -MAX_ANGLE_DELTA, MAX_ANGLE_DELTA)
                        delta_roll = clamp(delta_roll, -MAX_ANGLE_DELTA, MAX_ANGLE_DELTA)

                        # Convert to motor positions
                        # Yaw (좌우 회전): +y → rotate (ID 5)
                        rotate_pos = INITIAL_ROTATE + int(delta_yaw * SCALE_YAW)
                        # Pitch (위아래 까딱): +x → nod (ID 6)
                        nod_pos = INITIAL_NOD + int(delta_pitch * SCALE_PITCH)
                        # Roll (좌우 까딱): +z → tilt (ID 4) - 부호 반대
                        tilt_pos = INITIAL_TILT - int(delta_roll * SCALE_ROLL)

                        rotate_pos = clamp(rotate_pos, MIN_POS, MAX_POS)
                        nod_pos = clamp(nod_pos, MIN_POS, MAX_POS)
                        tilt_pos = clamp(tilt_pos, MIN_POS, MAX_POS)

                        # Send commands
                        controller.command_positions({
                            "nod": nod_pos,
                            "rotate": rotate_pos,
                            "tilt": tilt_pos
                        })

                except Exception as e:
                    if not stop_event.is_set():
                        print(f"[Motor Control] Error: {e}", file=sys.stderr)
                    break

    except Exception as e:
        print(f"[Motor Control] Thread error: {e}", file=sys.stderr)
    finally:
        server.close()
        print("[Motor Control] Thread stopped")


def camera_thread(stop_event):
    """Thread for displaying USB camera feed."""
    print(f"[Camera] Opening camera {CAMERA_INDEX}...")

    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print(f"[Camera] Error: Cannot open camera {CAMERA_INDEX}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"[Camera] Camera opened: {actual_width}x{actual_height}")
    print("[Camera] Press 'q' in camera window to quit")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    # Set window size (1.5x original resolution)
    cv2.resizeWindow(WINDOW_NAME, int(actual_width * 1), int(actual_height * 1))

    try:
        while not stop_event.is_set():
            ret, frame = cap.read()

            if not ret:
                print("[Camera] Failed to read frame")
                break

            cv2.imshow(WINDOW_NAME, frame)

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[Camera] Quit key pressed")
                stop_event.set()
                break

    except Exception as e:
        print(f"[Camera] Error: {e}", file=sys.stderr)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[Camera] Thread stopped")


def main():
    print("=" * 60)
    print("Neck Control with Camera Viewer")
    print("=" * 60)
    print(f"Motor control port: {UDP_PORT}")
    print(f"Camera index: {CAMERA_INDEX}")
    print()

    # Initialize motor controller
    controller = STS3215Controller(port="/dev/ttyUSB0")
    controller.open()
    controller.enable_torque(True)

    print("Initial motor positions:")
    print(f"  nod (ID 6):    {INITIAL_NOD} (Up/Down nod)")
    print(f"  rotate (ID 5): {INITIAL_ROTATE} (Left/Right rotation)")
    print(f"  tilt (ID 4):   {INITIAL_TILT} (Left/Right tilt - FIXED)")
    print()

    # Create stop event for threads
    stop_event = threading.Event()

    # Start threads
    motor_thread = threading.Thread(target=motor_control_thread, args=(controller, stop_event))
    cam_thread = threading.Thread(target=camera_thread, args=(stop_event,))

    motor_thread.start()
    cam_thread.start()

    try:
        # Wait for threads to finish
        while motor_thread.is_alive() or cam_thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nStopping...")
        stop_event.set()

    # Wait for threads to finish
    motor_thread.join(timeout=2)
    cam_thread.join(timeout=2)

    # Cleanup
    print("\nDisabling torque and closing...")
    controller.enable_torque(False)
    controller.close()
    print("Done!")


if __name__ == "__main__":
    main()
