from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup():
    # Launch arguments
    rs_type = LaunchConfiguration("rs_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    cabinet = LaunchConfiguration("cabinet")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("hiwin_rs4_moveit_config"),
                    "config",
                    "rs4.urdf.xacro",
                ]
            ),
            " ",
            "name:=",
            "rs_type",
            " ",
            "rs_type:=",
            rs_type,
            " ",
            "cabinet:=",
            cabinet,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "robot_ip:=",
            robot_ip,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Launch robot_state_publisher to publish TF frames
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": False},
        ],
    )

    # Path to the controller configuration file
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hiwin_driver"), "config", "rs4_controllers.yaml"]
    )

    # Launch ros2_control node with the controller configuration
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
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
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
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
        ),
    )

    # Path to RViz config file
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("hiwin_description"), "rviz", "view_robot.rviz"]
    )

    # Launch RViz if enabled via launch argument
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
    )

    # Delay RViz until joint_state_broadcaster is ready
    rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_after_jsb_spawner,
        rviz_after_joint_state_broadcaster_spawner,
        gpio_controller_after_robot_controller_spawner,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare launch arguments with default values and descriptions
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rs_type",
            description="Typo/series of used RS robot.",
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
            default_value="false",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Initially loaded robot controller.",
            choices=[
                "hiwin_controller",
                "joint_trajectory_controller",
            ],
        )
    )

    return LaunchDescription(declared_arguments + launch_setup())
