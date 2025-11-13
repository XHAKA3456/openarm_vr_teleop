#!/usr/bin/env python3
"""
Keyboard input for MoveIt Servo - OpenArm Bimanual version
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog
from control_msgs.action import GripperCommand
import sys
import termios
import tty


class KeyboardServoBoth(Node):
    def __init__(self):
        super().__init__('keyboard_servo_both')

        # Publishers for LEFT arm
        self.twist_pub_left = self.create_publisher(
            TwistStamped,
            '/left/servo_node/delta_twist_cmds',
            10
        )
        self.joint_pub_left = self.create_publisher(
            JointJog,
            '/left/servo_node/delta_joint_cmds',
            10
        )

        # Publishers for RIGHT arm
        self.twist_pub_right = self.create_publisher(
            TwistStamped,
            '/right/servo_node/delta_twist_cmds',
            10
        )
        self.joint_pub_right = self.create_publisher(
            JointJog,
            '/right/servo_node/delta_joint_cmds',
            10
        )

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

        # Subscribe to joint states to track gripper positions
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Frames
        self.frame_left = 'openarm_left_hand'
        self.frame_right = 'openarm_right_hand'

        # Joint control state
        self.active_arm = 'left'  # 'left' or 'right' for joint control
        self.joint_vel_cmd = 1.0

        # Gripper control
        self.gripper_max_position = 0.044  # Fully open (44mm)
        self.gripper_min_position = 0.0    # Fully closed
        self.gripper_left_position = 0.022  # Start at middle
        self.gripper_right_position = 0.022  # Start at middle
        self.gripper_step = 0.002  # 2mm per keypress

        self.print_instructions()

    def print_instructions(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("Keyboard Servo Control for OpenArm BIMANUAL")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        self.get_logger().info("LEFT ARM (WASD-based):")
        self.get_logger().info("  Translation: W/S (X), A/D (Y), Q/E (Z)")
        self.get_logger().info("  Rotation: T/G (Roll), F/H (Pitch), R/Y (Yaw)")
        self.get_logger().info("  Gripper: Z (open), X (close)")
        self.get_logger().info("")
        self.get_logger().info("RIGHT ARM (Arrow-based):")
        self.get_logger().info("  Translation: ↑/↓ (X), ←/→ (Y), ./; (Z)")
        self.get_logger().info("  Rotation: I/K (Roll), U/O (Pitch), J/L (Yaw)")
        self.get_logger().info("  Gripper: N (open), M (close)")
        self.get_logger().info("")
        self.get_logger().info("JOINT CONTROL:")
        self.get_logger().info("  Tab: Switch active arm (LEFT ↔ RIGHT)")
        self.get_logger().info("  1-7: Control selected joint of active arm")
        self.get_logger().info("  B: Reverse joint velocity direction")
        self.get_logger().info("")
        self.get_logger().info("0 (zero): Quit")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Active arm for joint control: {self.active_arm.upper()}")
        self.get_logger().info("=" * 60)

    def joint_state_callback(self, msg):
        """Update current gripper positions from joint states"""
        try:
            idx_left = msg.name.index('openarm_left_finger_joint1')
            self.gripper_left_position = msg.position[idx_left]
        except (ValueError, IndexError):
            pass

        try:
            idx_right = msg.name.index('openarm_right_finger_joint1')
            self.gripper_right_position = msg.position[idx_right]
        except (ValueError, IndexError):
            pass

    def get_key(self):
        """Read a single keypress from stdin"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # Arrow keys start with escape
                ch += sys.stdin.read(2)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def send_gripper_command(self, arm, delta):
        """Send gripper command via action client with incremental position"""
        if arm == 'left':
            current_position = self.gripper_left_position
            action_client = self.gripper_action_client_left
        else:
            current_position = self.gripper_right_position
            action_client = self.gripper_action_client_right

        # Calculate new position
        new_position = current_position + delta

        # Clamp to valid range
        new_position = max(self.gripper_min_position, min(self.gripper_max_position, new_position))

        # Send command
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = new_position
        goal_msg.command.max_effort = 100.0

        self.get_logger().info(f"[{arm.upper()} Gripper] {current_position:.4f}m -> {new_position:.4f}m (delta={delta:+.4f}m)")

        # Update internal position
        if arm == 'left':
            self.gripper_left_position = new_position
        else:
            self.gripper_right_position = new_position

        # Send goal (non-blocking)
        action_client.send_goal_async(goal_msg)

    def run(self):
        """Main loop to read keyboard and publish commands"""
        while rclpy.ok():
            # Spin once to process callbacks
            rclpy.spin_once(self, timeout_sec=0.0)

            key = self.get_key()

            # Create message containers
            twist_msg_left = TwistStamped()
            twist_msg_right = TwistStamped()
            joint_msg = JointJog()
            publish_left = False
            publish_right = False
            publish_joint = False

            # LEFT ARM - Cartesian control (WASD-based)
            if key == 'w' or key == 'W':
                twist_msg_left.twist.linear.x = 1.0
                publish_left = True
            elif key == 's' or key == 'S':
                twist_msg_left.twist.linear.x = -1.0
                publish_left = True
            elif key == 'a' or key == 'A':
                twist_msg_left.twist.linear.y = -1.0
                publish_left = True
            elif key == 'd' or key == 'D':
                twist_msg_left.twist.linear.y = 1.0
                publish_left = True
            elif key == 'q' or key == 'Q':
                twist_msg_left.twist.linear.z = 1.0
                publish_left = True
            elif key == 'e' or key == 'E':
                twist_msg_left.twist.linear.z = -1.0
                publish_left = True
            elif key == 't' or key == 'T':
                twist_msg_left.twist.angular.x = 1.0
                publish_left = True
            elif key == 'g' or key == 'G':
                twist_msg_left.twist.angular.x = -1.0
                publish_left = True
            elif key == 'f' or key == 'F':
                twist_msg_left.twist.angular.y = 1.0
                publish_left = True
            elif key == 'h' or key == 'H':
                twist_msg_left.twist.angular.y = -1.0
                publish_left = True
            elif key == 'r' or key == 'R':
                twist_msg_left.twist.angular.z = 1.0
                publish_left = True
            elif key == 'y' or key == 'Y':
                twist_msg_left.twist.angular.z = -1.0
                publish_left = True
            # LEFT gripper
            elif key == 'z' or key == 'Z':
                self.send_gripper_command('left', self.gripper_step)
            elif key == 'x' or key == 'X':
                self.send_gripper_command('left', -self.gripper_step)

            # RIGHT ARM - Cartesian control (Arrow-based)
            elif key == '\x1b[A':  # UP arrow
                twist_msg_right.twist.linear.x = 1.0
                publish_right = True
            elif key == '\x1b[B':  # DOWN arrow
                twist_msg_right.twist.linear.x = -1.0
                publish_right = True
            elif key == '\x1b[C':  # RIGHT arrow
                twist_msg_right.twist.linear.y = 1.0
                publish_right = True
            elif key == '\x1b[D':  # LEFT arrow
                twist_msg_right.twist.linear.y = -1.0
                publish_right = True
            elif key == ';':
                twist_msg_right.twist.linear.z = 1.0
                publish_right = True
            elif key == '.':
                twist_msg_right.twist.linear.z = -1.0
                publish_right = True
            elif key == 'i' or key == 'I':
                twist_msg_right.twist.angular.x = 1.0
                publish_right = True
            elif key == 'k' or key == 'K':
                twist_msg_right.twist.angular.x = -1.0
                publish_right = True
            elif key == 'u' or key == 'U':
                twist_msg_right.twist.angular.y = 1.0
                publish_right = True
            elif key == 'o' or key == 'O':
                twist_msg_right.twist.angular.y = -1.0
                publish_right = True
            elif key == 'j' or key == 'J':
                twist_msg_right.twist.angular.z = 1.0
                publish_right = True
            elif key == 'l' or key == 'L':
                twist_msg_right.twist.angular.z = -1.0
                publish_right = True
            # RIGHT gripper
            elif key == 'n' or key == 'N':
                self.send_gripper_command('right', self.gripper_step)
            elif key == 'm' or key == 'M':
                self.send_gripper_command('right', -self.gripper_step)

            # JOINT CONTROL
            elif key == '\t':  # Tab key
                self.active_arm = 'right' if self.active_arm == 'left' else 'left'
                self.get_logger().info(f"→ Active arm switched to: {self.active_arm.upper()}")
            elif key == 'b' or key == 'B':
                self.joint_vel_cmd *= -1
                self.get_logger().info(f"→ Joint velocity direction reversed: {self.joint_vel_cmd}")
            elif key in ['1', '2', '3', '4', '5', '6', '7']:
                joint_num = int(key)
                prefix = 'openarm_left' if self.active_arm == 'left' else 'openarm_right'
                joint_msg.joint_names = [f'{prefix}_joint{joint_num}']
                joint_msg.velocities = [self.joint_vel_cmd]
                publish_joint = True

            # QUIT
            elif key == '0':
                self.get_logger().info("Exiting keyboard control...")
                break

            # Publish LEFT arm Twist
            if publish_left:
                twist_msg_left.header.stamp = self.get_clock().now().to_msg()
                twist_msg_left.header.frame_id = self.frame_left
                self.twist_pub_left.publish(twist_msg_left)
                self.get_logger().info(
                    f"[LEFT Twist] key={repr(key)} | "
                    f"linear=({twist_msg_left.twist.linear.x:.1f}, {twist_msg_left.twist.linear.y:.1f}, {twist_msg_left.twist.linear.z:.1f}) | "
                    f"angular=({twist_msg_left.twist.angular.x:.1f}, {twist_msg_left.twist.angular.y:.1f}, {twist_msg_left.twist.angular.z:.1f})"
                )

            # Publish RIGHT arm Twist
            if publish_right:
                twist_msg_right.header.stamp = self.get_clock().now().to_msg()
                twist_msg_right.header.frame_id = self.frame_right
                self.twist_pub_right.publish(twist_msg_right)
                self.get_logger().info(
                    f"[RIGHT Twist] key={repr(key)} | "
                    f"linear=({twist_msg_right.twist.linear.x:.1f}, {twist_msg_right.twist.linear.y:.1f}, {twist_msg_right.twist.linear.z:.1f}) | "
                    f"angular=({twist_msg_right.twist.angular.x:.1f}, {twist_msg_right.twist.angular.y:.1f}, {twist_msg_right.twist.angular.z:.1f})"
                )

            # Publish Joint command
            if publish_joint:
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.header.frame_id = 'openarm_body_link0'
                # Publish to the correct arm's topic
                if self.active_arm == 'left':
                    self.joint_pub_left.publish(joint_msg)
                else:
                    self.joint_pub_right.publish(joint_msg)
                self.get_logger().info(
                    f"[{self.active_arm.upper()} Joint] key={repr(key)} | joint={joint_msg.joint_names[0]} | vel={joint_msg.velocities[0]:.3f}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServoBoth()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
