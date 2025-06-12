from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup():
    # Launch arguments
    rs_type = LaunchConfiguration("rs_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    cabinet = LaunchConfiguration("cabinet")
    launch_rviz = LaunchConfiguration("launch_rviz")
    tf_prefix = LaunchConfiguration("tf_prefix")

    # Build the MoveIt configuration from xacro and YAML files
    moveit_config = (
        MoveItConfigsBuilder("hiwin_rs4")
        .robot_description(
            file_path="config/rs4.urdf.xacro",
            mappings={
                "name": "hiwin",
                "rs_type": rs_type,
                "use_fake_hardware": use_fake_hardware,
                "cabinet": cabinet,
                "robot_ip": robot_ip,
                "tf_prefix": tf_prefix,
            },
        )
        .robot_description_semantic(Path("config") / "rs4.srdf")
        .robot_description_kinematics(Path("config") / "kinematics.yaml")
        .trajectory_execution(Path("config") / "moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Launch robot_state_publisher to publish TF frames
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
    )

    # Path to the controller configuration file
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hiwin_rs4_moveit_config"), "config", "ros2_controllers.yaml"]
    )

    # Launch ros2_control node with the controller configuration
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
        ],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    # Spawner for joint_state_broadcaster (publishes joint_states topic)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Spawner for the main manipulator controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "manipulator_controller",
            "-c",
            "/controller_manager",
        ],
    )

    # Delay launching manipulator controller until joint_state_broadcaster is ready
    robot_controller_after_jsb_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Spawner for GPIO controller (e.g., for digital IO)
    gpio_controller_spawner = Node(
        package="controller_manager",
        condition=IfCondition(PythonExpression(['"', cabinet, '" == "ecat"'])),
        executable="spawner",
        arguments=[
            "gpio_controller",
            "-c",
            "/controller_manager",
            "--param-file",
            PathJoinSubstitution(
                [FindPackageShare("hiwin_driver"), "config", "gpio_controller.yaml"]
            ),
        ],
    )

    # Delay launching GPIO controller until manipulator controller is ready
    gpio_controller_after_robot_controller_spawner = RegisterEventHandler(
        condition=IfCondition(PythonExpression(['"', cabinet, '" == "ecat"'])),
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[gpio_controller_spawner],
        )
    )

    # Launch MoveIt move_group node (main planning pipeline)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    # Delay launching move_group until manipulator controller is ready
    move_group_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    # Path to RViz config file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("hiwin_rs4_moveit_config"), "config", "moveit.rviz"]
    )

    # Launch RViz if enabled via launch argument
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Delay RViz until joint_state_broadcaster is ready
    rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # List of all nodes and event handlers to launch
    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        move_group_after_robot_controller_spawner,
        robot_controller_after_jsb_spawner,
        gpio_controller_after_robot_controller_spawner,
        rviz_after_joint_state_broadcaster_spawner,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare launch arguments with default values and descriptions
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rs_type",
            description="Type/series of used RS robot.",
            choices=[
                "rs405_400_200",
                "rs405_500_400",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use mock hardware that mirrors commands to states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="0.0.0.0",
            description="IP address of the physical robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cabinet",
            default_value="rc4-a",
            description="HIWIN robot control cabinet model.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Whether to launch RViz for visualization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description=(
                "Prefix for joint names. Required for multi-robot setup. "
                "Must match joint names in controller YAML config."
            ),
        )
    )

    return LaunchDescription(declared_arguments + launch_setup())
