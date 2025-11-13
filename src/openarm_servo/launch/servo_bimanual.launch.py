import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("openarm_bimanual")
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro",
            mappings={
                "ros2_control": "true",
                "use_fake_hardware": "true",
                "bimanual": "true",
            }
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Get parameters for the Servo nodes
    servo_yaml_left = load_yaml("openarm_servo", "config/openarm_left_simulated_config.yaml")
    servo_params_left = {"moveit_servo": servo_yaml_left}

    servo_yaml_right = load_yaml("openarm_servo", "config/openarm_right_simulated_config.yaml")
    servo_params_right = {"moveit_servo": servo_yaml_right}

    # RViz
    rviz_config_file = (
        get_package_share_directory("openarm_servo") + "/config/openarm_servo_bimanual.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("openarm_servo"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Left arm controller
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller", "-c", "/controller_manager"],
    )

    # Right arm controller
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "-c", "/controller_manager"],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    # Launch TWO standalone Servo nodes with namespaces
    # Left arm servo node
    servo_node_left = Node(
        package="moveit_servo",
        executable="servo_node_main",
        namespace="left",
        parameters=[
            servo_params_left,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    # Right arm servo node
    servo_node_right = Node(
        package="moveit_servo",
        executable="servo_node_main",
        namespace="right",
        parameters=[
            servo_params_right,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    # Start servo service calls (delayed to allow nodes to initialize)
    start_servo_left = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/left/servo_node/start_servo', 'std_srvs/srv/Trigger'],
                output='screen'
            )
        ]
    )

    start_servo_right = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/right/servo_node/start_servo', 'std_srvs/srv/Trigger'],
                output='screen'
            )
        ]
    )

    return LaunchDescription(
        [
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            left_gripper_controller_spawner,
            right_arm_controller_spawner,
            right_gripper_controller_spawner,
            servo_node_left,
            servo_node_right,
            container,
            start_servo_left,
            start_servo_right,
        ]
    )
