#!/usr/bin/env python3
"""
Quest VR Teleoperation for OpenArm Bimanual
Alternating stages: Left homing → Right homing → Left calibration → Right calibration → Teleoperation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.action import GripperCommand
import time
import socket
import json
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R


class QuestServoBoth(Node):
    def __init__(self, host='0.0.0.0', port=5454):
        super().__init__('quest_servo_both')

        # Publishers - LEFT ARM (with namespace)
        self.joint_pub_left = self.create_publisher(JointJog, '/left/servo_node/delta_joint_cmds', 10)
        self.twist_pub_left = self.create_publisher(TwistStamped, '/left/servo_node/delta_twist_cmds', 10)

        # Publishers - RIGHT ARM (with namespace)
        self.joint_pub_right = self.create_publisher(JointJog, '/right/servo_node/delta_joint_cmds', 10)
        self.twist_pub_right = self.create_publisher(TwistStamped, '/right/servo_node/delta_twist_cmds', 10)

        # Subscriber
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Gripper action clients
        self.gripper_action_client_left = ActionClient(
            self,
            GripperCommand,
            '/left_gripper_controller/gripper_cmd'
        )
        self.gripper_action_client_right = ActionClient(
            self,
            GripperCommand,
            '/right_gripper_controller/gripper_cmd'
        )

        # Joint positions
        self.current_positions = {}

        # Home positions
        self.home_joint4 = 1.58  # Target for both arms

        # State machine
        # 0: waiting, 1: left j4, 2: right j4, 3: left calibration, 4: right calibration, 5: teleoperation
        self.stage = 0
        self.joint_vel = 2.0
        self.threshold = 0.01

        # Quest data (socket thread updates this)
        self.latest_quest_data = None
        self.data_lock = threading.Lock()

        # Calibration - LEFT
        self.calibration_samples_left = []
        self.calibration_duration = 2.5  # 2.5 seconds
        self.calibration_start_time_left = None
        self.reference_position_left = None
        self.reference_rotation_left = None

        # Calibration - RIGHT
        self.calibration_samples_right = []
        self.calibration_start_time_right = None
        self.reference_position_right = None
        self.reference_rotation_right = None

        # Teleoperation - LEFT
        self.prev_position_left = None
        self.prev_rotation_left = None
        self.prev_timestamp_left = None
        self.linear_scale = 5.0
        self.angular_scale = 5.0

        # Teleoperation - RIGHT
        self.prev_position_right = None
        self.prev_rotation_right = None
        self.prev_timestamp_right = None

        # Gripper control
        self.gripper_min_position = 0.0    # Fully closed
        self.gripper_max_position = 0.044  # Fully open (44mm)
        self.prev_trigger_left = None
        self.prev_trigger_right = None

        # Socket server
        self.host = host
        self.port = port
        self.running = True
        self.socket_thread = threading.Thread(target=self.socket_server, daemon=True)
        self.socket_thread.start()

        self.get_logger().info("=== Quest Teleoperation BIMANUAL ===")
        self.get_logger().info("Stage 0: Waiting 3s")
        self.get_logger().info("Stage 1: LEFT J4 homing to 1.58")
        self.get_logger().info("Stage 2: RIGHT J4 homing to 1.58")
        self.get_logger().info("Stage 3: LEFT calibration (2.5s)")
        self.get_logger().info("Stage 4: RIGHT calibration (2.5s)")
        self.get_logger().info("Stage 5: Teleoperation (BOTH)")

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_positions[name] = position

    def socket_server(self):
        """Socket server thread - receives Quest data"""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server.bind((self.host, self.port))
            server.listen(1)
            self.get_logger().info(f"[SOCKET] Listening on {self.host}:{self.port}")

            conn, addr = server.accept()
            self.get_logger().info(f"[SOCKET] Connected: {addr}")

            buffer = ""
            while self.running and rclpy.ok():
                data = conn.recv(1024)
                if not data:
                    break

                buffer += data.decode()
                lines = buffer.split('\n')

                for line in lines[:-1]:
                    if line.strip():
                        try:
                            quest_data = json.loads(line)
                            with self.data_lock:
                                self.latest_quest_data = quest_data
                        except json.JSONDecodeError:
                            pass

                buffer = lines[-1]

        except Exception as e:
            self.get_logger().error(f"[SOCKET] Error: {e}")
        finally:
            server.close()

    def controller_to_openarm_position(self, ctrl_pos):
        """Transform Quest controller position to OpenArm frame"""
        # Swap: [x, y, z] -> [y, x, z]
        return np.array([ctrl_pos[1], ctrl_pos[0], ctrl_pos[2]])

    def controller_to_openarm_rotation(self, ctrl_quat):
        """Transform Quest controller rotation to OpenArm frame"""
        # Convert to rotation matrix
        ctrl_rot = R.from_quat(ctrl_quat)
        mat = ctrl_rot.as_matrix()

        # Swap columns: [x, y, z] -> [y, x, z]
        mat_swapped = mat[:, [1, 0, 2]]

        # Swap rows: [x, y, z] -> [y, x, z]
        mat_swapped = mat_swapped[[1, 0, 2], :]

        # Convert back to quaternion
        return R.from_matrix(mat_swapped).as_quat()

    def send_gripper_command(self, arm, trigger_value):
        """Send gripper command based on trigger value (0~1)"""
        # INVERTED: trigger 0 (release) = open, trigger 1 (press) = close
        inverted_trigger = 1.0 - trigger_value

        # Map inverted trigger (0~1) to gripper position (0.0~0.044)
        gripper_position = self.gripper_min_position + \
                          (inverted_trigger * (self.gripper_max_position - self.gripper_min_position))

        # Create and send goal
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = gripper_position
        goal_msg.command.max_effort = 100.0

        # Send goal (non-blocking)
        if arm == 'left':
            self.gripper_action_client_left.send_goal_async(goal_msg)
        else:
            self.gripper_action_client_right.send_goal_async(goal_msg)

    def run(self):
        """Main loop"""
        # Stage 0: Wait 3 seconds
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.01)

        # Check joint states received
        if 'openarm_left_joint4' not in self.current_positions or 'openarm_right_joint4' not in self.current_positions:
            self.get_logger().error("No joint states!")
            return

        # Start homing
        self.get_logger().info(f"[LEFT HOMING] j4: {self.current_positions.get('openarm_left_joint4', 0):.3f} -> {self.home_joint4:.3f}")
        self.stage = 1

        msg_count = 0

        # Main loop
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            # Stage 1: Homing LEFT joint4
            if self.stage == 1:
                current = self.current_positions.get('openarm_left_joint4', 0)
                delta = self.home_joint4 - current

                if abs(delta) < self.threshold:
                    self.get_logger().info(f"[LEFT HOMING] j4 done ({current:.3f})")
                    self.get_logger().info(f"[RIGHT HOMING] j4: {self.current_positions.get('openarm_right_joint4', 0):.3f} -> {self.home_joint4:.3f}")
                    self.stage = 2
                    msg_count = 0
                else:
                    vel = self.joint_vel if delta > 0 else -self.joint_vel
                    msg = JointJog()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'openarm_body_link0'
                    msg.joint_names = ['openarm_left_joint4']
                    msg.velocities = [vel]
                    self.joint_pub_left.publish(msg)

            # Stage 2: Homing RIGHT joint4
            elif self.stage == 2:
                current = self.current_positions.get('openarm_right_joint4', 0)
                delta = self.home_joint4 - current

                if abs(delta) < self.threshold:
                    self.get_logger().info(f"[RIGHT HOMING] j4 done ({current:.3f})")

                    # Open both grippers after homing
                    self.get_logger().info("[GRIPPER] Opening both grippers...")
                    self.send_gripper_command('left', 0.0)   # 0.0 = fully open (inverted)
                    self.send_gripper_command('right', 0.0)

                    self.get_logger().info("[LEFT CALIBRATION] Waiting for Quest data...")
                    self.stage = 3
                    self.calibration_start_time_left = None
                    self.calibration_samples_left = []
                    msg_count = 0
                else:
                    vel = self.joint_vel if delta > 0 else -self.joint_vel
                    msg = JointJog()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'openarm_body_link0'
                    msg.joint_names = ['openarm_right_joint4']
                    msg.velocities = [vel]
                    self.joint_pub_right.publish(msg)

            # Stage 3: LEFT Calibration (collect samples for 2.5 seconds)
            elif self.stage == 3:
                # Log waiting status every 1 second
                if self.calibration_start_time_left is None:
                    msg_count += 1
                    if msg_count % 100 == 0:
                        self.get_logger().info("[LEFT CALIBRATION] Waiting for Quest connection...")

                with self.data_lock:
                    if self.latest_quest_data:
                        left = self.latest_quest_data.get('left', {})
                        if left.get('enabled', False):
                            # First data received - start calibration timer
                            if self.calibration_start_time_left is None:
                                self.calibration_start_time_left = time.time()
                                msg_count = 0
                                self.get_logger().info("[LEFT CALIBRATION] Quest connected! Starting 2.5s calibration...")

                            pos = left.get('position', {})
                            rot = left.get('rotation', {})

                            ctrl_pos = np.array([pos['x'], pos['y'], pos['z']])
                            ctrl_quat = np.array([rot['x'], rot['y'], rot['z'], rot['w']])

                            # Transform to OpenArm frame
                            openarm_pos = self.controller_to_openarm_position(ctrl_pos)
                            openarm_quat = self.controller_to_openarm_rotation(ctrl_quat)

                            self.calibration_samples_left.append((openarm_pos, openarm_quat))

                # Check if calibration time elapsed
                if self.calibration_start_time_left is not None:
                    elapsed = time.time() - self.calibration_start_time_left
                else:
                    elapsed = 0

                if elapsed >= self.calibration_duration and self.calibration_start_time_left is not None:
                    if len(self.calibration_samples_left) > 0:
                        # Calculate average
                        positions = np.array([s[0] for s in self.calibration_samples_left])
                        quaternions = np.array([s[1] for s in self.calibration_samples_left])

                        self.reference_position_left = np.mean(positions, axis=0)

                        # Average quaternions (simple mean, not geodesic)
                        self.reference_rotation_left = np.mean(quaternions, axis=0)
                        self.reference_rotation_left /= np.linalg.norm(self.reference_rotation_left)

                        self.get_logger().info(f"[LEFT CALIBRATION] Done! Samples: {len(self.calibration_samples_left)}")
                        self.get_logger().info("[RIGHT CALIBRATION] Waiting for Quest data...")
                        self.stage = 4
                        self.calibration_start_time_right = None
                        self.calibration_samples_right = []
                        msg_count = 0
                    else:
                        self.get_logger().error("[LEFT CALIBRATION] No data received!")
                        return

            # Stage 4: RIGHT Calibration (collect samples for 2.5 seconds)
            elif self.stage == 4:
                # Log waiting status every 1 second
                if self.calibration_start_time_right is None:
                    msg_count += 1
                    if msg_count % 100 == 0:
                        self.get_logger().info("[RIGHT CALIBRATION] Waiting for Quest connection...")

                with self.data_lock:
                    if self.latest_quest_data:
                        right = self.latest_quest_data.get('right', {})
                        if right.get('enabled', False):
                            # First data received - start calibration timer
                            if self.calibration_start_time_right is None:
                                self.calibration_start_time_right = time.time()
                                msg_count = 0
                                self.get_logger().info("[RIGHT CALIBRATION] Quest connected! Starting 2.5s calibration...")

                            pos = right.get('position', {})
                            rot = right.get('rotation', {})

                            ctrl_pos = np.array([pos['x'], pos['y'], pos['z']])
                            ctrl_quat = np.array([rot['x'], rot['y'], rot['z'], rot['w']])

                            # Transform to OpenArm frame
                            openarm_pos = self.controller_to_openarm_position(ctrl_pos)
                            openarm_quat = self.controller_to_openarm_rotation(ctrl_quat)

                            self.calibration_samples_right.append((openarm_pos, openarm_quat))

                # Check if calibration time elapsed
                if self.calibration_start_time_right is not None:
                    elapsed = time.time() - self.calibration_start_time_right
                else:
                    elapsed = 0

                if elapsed >= self.calibration_duration and self.calibration_start_time_right is not None:
                    if len(self.calibration_samples_right) > 0:
                        # Calculate average
                        positions = np.array([s[0] for s in self.calibration_samples_right])
                        quaternions = np.array([s[1] for s in self.calibration_samples_right])

                        self.reference_position_right = np.mean(positions, axis=0)

                        # Average quaternions (simple mean, not geodesic)
                        self.reference_rotation_right = np.mean(quaternions, axis=0)
                        self.reference_rotation_right /= np.linalg.norm(self.reference_rotation_right)

                        self.get_logger().info(f"[RIGHT CALIBRATION] Done! Samples: {len(self.calibration_samples_right)}")
                        self.get_logger().info("[TELEOPERATION] Ready! Move both controllers.")
                        self.stage = 5
                    else:
                        self.get_logger().error("[RIGHT CALIBRATION] No data received!")
                        return

            # Stage 5: Teleoperation (BOTH ARMS)
            elif self.stage == 5:
                self.publish_quest_twist_left()
                self.publish_quest_twist_right()
                self.publish_gripper_command_left()
                self.publish_gripper_command_right()

            time.sleep(0.01)

    def publish_quest_twist_left(self):
        """Calculate and publish twist from Quest controller - LEFT ARM"""
        with self.data_lock:
            if not self.latest_quest_data:
                return
            quest_data = self.latest_quest_data.copy()

        left = quest_data.get('left', {})
        if not left.get('enabled', False):
            return

        timestamp = quest_data.get('timestamp', 0.0)
        pos = left.get('position', {})
        rot = left.get('rotation', {})

        ctrl_pos = np.array([pos['x'], pos['y'], pos['z']])
        ctrl_quat = np.array([rot['x'], rot['y'], rot['z'], rot['w']])

        # Transform to OpenArm frame
        current_pos = self.controller_to_openarm_position(ctrl_pos)
        current_quat = self.controller_to_openarm_rotation(ctrl_quat)

        # Initialize on first data
        if self.prev_position_left is None:
            self.prev_position_left = current_pos
            self.prev_rotation_left = current_quat
            self.prev_timestamp_left = timestamp
            self.get_logger().info("[LEFT TELEOPERATION] Initialized!")
            return

        # Calculate time delta
        dt = timestamp - self.prev_timestamp_left
        if dt <= 0 or dt > 1.0:  # Skip invalid dt
            self.prev_timestamp_left = timestamp
            return

        # Calculate velocity (position change over time)
        linear_vel = (current_pos - self.prev_position_left) / dt

        # Calculate angular velocity
        prev_rot = R.from_quat(self.prev_rotation_left)
        curr_rot = R.from_quat(current_quat)
        delta_rot = curr_rot * prev_rot.inv()
        angular_vel = delta_rot.as_rotvec() / dt

        # Create twist message
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'openarm_left_hand'

        twist.twist.linear.x = float(linear_vel[0] * self.linear_scale)
        twist.twist.linear.y = float(linear_vel[1] * self.linear_scale)
        twist.twist.linear.z = float(linear_vel[2] * self.linear_scale)

        twist.twist.angular.x = float(angular_vel[0] * self.angular_scale)
        twist.twist.angular.y = float(angular_vel[1] * self.angular_scale)
        twist.twist.angular.z = float(angular_vel[2] * self.angular_scale)

        self.twist_pub_left.publish(twist)

        # Update previous values
        self.prev_position_left = current_pos
        self.prev_rotation_left = current_quat
        self.prev_timestamp_left = timestamp

    def publish_quest_twist_right(self):
        """Calculate and publish twist from Quest controller - RIGHT ARM"""
        with self.data_lock:
            if not self.latest_quest_data:
                return
            quest_data = self.latest_quest_data.copy()

        right = quest_data.get('right', {})
        if not right.get('enabled', False):
            return

        timestamp = quest_data.get('timestamp', 0.0)
        pos = right.get('position', {})
        rot = right.get('rotation', {})

        ctrl_pos = np.array([pos['x'], pos['y'], pos['z']])
        ctrl_quat = np.array([rot['x'], rot['y'], rot['z'], rot['w']])

        # Transform to OpenArm frame
        current_pos = self.controller_to_openarm_position(ctrl_pos)
        current_quat = self.controller_to_openarm_rotation(ctrl_quat)

        # Initialize on first data
        if self.prev_position_right is None:
            self.prev_position_right = current_pos
            self.prev_rotation_right = current_quat
            self.prev_timestamp_right = timestamp
            self.get_logger().info("[RIGHT TELEOPERATION] Initialized!")
            return

        # Calculate time delta
        dt = timestamp - self.prev_timestamp_right
        if dt <= 0 or dt > 1.0:  # Skip invalid dt
            self.prev_timestamp_right = timestamp
            return

        # Calculate velocity (position change over time)
        linear_vel = (current_pos - self.prev_position_right) / dt

        # Calculate angular velocity
        prev_rot = R.from_quat(self.prev_rotation_right)
        curr_rot = R.from_quat(current_quat)
        delta_rot = curr_rot * prev_rot.inv()
        angular_vel = delta_rot.as_rotvec() / dt

        # Create twist message
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'openarm_right_hand'

        twist.twist.linear.x = float(linear_vel[0] * self.linear_scale)
        twist.twist.linear.y = float(linear_vel[1] * self.linear_scale)
        twist.twist.linear.z = float(linear_vel[2] * self.linear_scale)

        twist.twist.angular.x = float(angular_vel[0] * self.angular_scale)
        twist.twist.angular.y = float(angular_vel[1] * self.angular_scale)
        twist.twist.angular.z = float(angular_vel[2] * self.angular_scale)

        self.twist_pub_right.publish(twist)

        # Update previous values
        self.prev_position_right = current_pos
        self.prev_rotation_right = current_quat
        self.prev_timestamp_right = timestamp

    def publish_gripper_command_left(self):
        """Publish gripper command for LEFT ARM based on trigger"""
        with self.data_lock:
            if not self.latest_quest_data:
                return
            quest_data = self.latest_quest_data.copy()

        left = quest_data.get('left', {})
        if not left.get('enabled', False):
            return

        trigger_value = left.get('trigger', 0.0)

        # Send command only when trigger changes
        if self.prev_trigger_left is None or abs(trigger_value - self.prev_trigger_left) > 0.01:
            self.send_gripper_command('left', trigger_value)
            self.prev_trigger_left = trigger_value

    def publish_gripper_command_right(self):
        """Publish gripper command for RIGHT ARM based on trigger"""
        with self.data_lock:
            if not self.latest_quest_data:
                return
            quest_data = self.latest_quest_data.copy()

        right = quest_data.get('right', {})
        if not right.get('enabled', False):
            return

        trigger_value = right.get('trigger', 0.0)

        # Send command only when trigger changes
        if self.prev_trigger_right is None or abs(trigger_value - self.prev_trigger_right) > 0.01:
            self.send_gripper_command('right', trigger_value)
            self.prev_trigger_right = trigger_value


def main(args=None):
    rclpy.init(args=args)
    node = QuestServoBoth()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
