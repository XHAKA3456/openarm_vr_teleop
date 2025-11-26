#!/usr/bin/env python3
"""
Complete neck control system:
1. Receives Meta Quest head tracking data and controls neck motors
2. Streams USB camera feed to Meta Quest (no PC display)
"""

import cv2
import json
import socket
import struct
import sys
import time
import threading
import queue
from controller import STS3215Controller

# Network settings
POSE_PORT = 5454        # Receive Quest pose data
VIDEO_PORT = 5656       # Send video to Quest (changed from 5556 due to conflict)
VIDEO_SERVER_IP = "0.0.0.0"

# Camera settings
CAMERA_INDEX = 2
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
JPEG_QUALITY = 70

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
    except (json.JSONDecodeError, KeyError):
        pass
    return None


def motor_control_thread(controller, stop_event):
    """Thread for receiving Quest data and controlling motors."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", POSE_PORT))
    server.listen(1)

    print(f"[Motor] Listening on TCP port {POSE_PORT}...")

    conn, addr = server.accept()
    print(f"[Motor] Connected: {addr}")

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
                        print("[Motor] Connection closed")
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
                                print("[Motor] Calibration started (3 seconds)...")

                            calibration_samples.append(euler)
                            elapsed = time.time() - calibration_start

                            if elapsed >= 3.0:
                                initial_yaw = sum(s['y'] for s in calibration_samples) / len(calibration_samples)
                                initial_pitch = sum(s['x'] for s in calibration_samples) / len(calibration_samples)
                                initial_roll = sum(s['z'] for s in calibration_samples) / len(calibration_samples)
                                print(f"[Motor] ✓ Calibrated!")
                                print(f"[Motor]   Yaw: {initial_yaw:.1f}° Pitch: {initial_pitch:.1f}° Roll: {initial_roll:.1f}°")
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
                        print(f"[Motor] Error: {e}")
                    break

    finally:
        server.close()
        print("[Motor] Thread stopped")


def video_stream_thread(frame_queue, stop_event):
    """Thread for streaming video to Quest."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((VIDEO_SERVER_IP, VIDEO_PORT))
    except OSError as e:
        print(f"[Video] Error binding to port {VIDEO_PORT}: {e}")
        return

    server.listen(1)
    print(f"[Video] Listening on port {VIDEO_PORT}...")

    try:
        conn, addr = server.accept()
        print(f"[Video] Connected: {addr}")

        frame_count = 0
        start_time = time.time()

        while not stop_event.is_set():
            try:
                frame = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if frame is None:
                continue

            # JPEG compression
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            result, jpeg = cv2.imencode('.jpg', frame, encode_param)

            if not result:
                continue

            jpeg_data = jpeg.tobytes()
            data_size = len(jpeg_data)

            try:
                # Send size (4 bytes, big-endian)
                size_bytes = struct.pack('>I', data_size)
                conn.sendall(size_bytes)

                # Send JPEG data
                conn.sendall(jpeg_data)

                frame_count += 1

                # FPS info
                elapsed = time.time() - start_time
                if elapsed >= 5.0:
                    fps = frame_count / elapsed
                    print(f"[Video] {fps:.1f} FPS | {data_size/1024:.1f} KB/frame")
                    frame_count = 0
                    start_time = time.time()

            except (BrokenPipeError, ConnectionResetError):
                print("[Video] Quest disconnected")
                break

    except Exception as e:
        print(f"[Video] Error: {e}")
    finally:
        server.close()
        print("[Video] Thread stopped")


def camera_thread(frame_queue, stop_event):
    """Thread for capturing camera frames."""
    print(f"[Camera] Opening camera {CAMERA_INDEX}...")

    # Try opening with V4L2 backend explicitly
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"[Camera] Failed with V4L2, trying default backend...")
        cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print(f"[Camera] Error: Cannot open camera {CAMERA_INDEX}")
        print(f"[Camera] Try different index: 0, 1, 2, 3...")
        stop_event.set()
        return

    # Set properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 20)

    # Verify camera is working
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if actual_width == 0 or actual_height == 0:
        print(f"[Camera] Error: Invalid resolution {actual_width}x{actual_height}")
        print(f"[Camera] Camera {CAMERA_INDEX} may not be a real camera device")
        cap.release()
        stop_event.set()
        return

    print(f"[Camera] Opened: {actual_width}x{actual_height}")

    try:
        while not stop_event.is_set():
            ret, frame = cap.read()

            if not ret:
                print("[Camera] Failed to read frame")
                time.sleep(0.1)
                continue

            # Put frame in queue for streaming
            try:
                frame_queue.put_nowait(frame)
            except queue.Full:
                # Drop frame if queue is full
                pass

    except Exception as e:
        print(f"[Camera] Error: {e}")
    finally:
        cap.release()
        print("[Camera] Thread stopped")


def main():
    print("=" * 60)
    print("Complete Neck Control System")
    print("=" * 60)
    print(f"Pose control port: {POSE_PORT}")
    print(f"Video stream port: {VIDEO_PORT}")
    print(f"Camera: {CAMERA_INDEX}")
    print()

    # Initialize motor controller
    controller = STS3215Controller(port="/dev/ttyUSB0")
    controller.open()
    controller.enable_torque(True)

    print("Motor positions:")
    print(f"  nod (ID 6):    {INITIAL_NOD} (Up/Down nod)")
    print(f"  rotate (ID 5): {INITIAL_ROTATE} (Left/Right rotation)")
    print(f"  tilt (ID 4):   {INITIAL_TILT} (Left/Right tilt - FIXED)")
    print()

    # Create stop event and frame queue
    stop_event = threading.Event()
    frame_queue = queue.Queue(maxsize=2)

    # Start threads
    motor_thread = threading.Thread(target=motor_control_thread, args=(controller, stop_event), daemon=True)
    video_thread = threading.Thread(target=video_stream_thread, args=(frame_queue, stop_event), daemon=True)
    cam_thread = threading.Thread(target=camera_thread, args=(frame_queue, stop_event), daemon=True)

    motor_thread.start()
    video_thread.start()
    cam_thread.start()

    print("All threads started. Press Ctrl+C to stop.")
    print()

    try:
        while motor_thread.is_alive() or video_thread.is_alive() or cam_thread.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n\nStopping...")
        stop_event.set()

    # Wait for threads
    motor_thread.join(timeout=2)
    video_thread.join(timeout=2)
    cam_thread.join(timeout=2)

    # Cleanup
    print("\nShutting down...")
    controller.enable_torque(False)
    controller.close()
    print("✓ Done")


if __name__ == "__main__":
    main()
