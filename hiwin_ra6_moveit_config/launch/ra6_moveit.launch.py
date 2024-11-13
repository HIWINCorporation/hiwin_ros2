from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup():
    ra_type = LaunchConfiguration("ra_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    cabinet = LaunchConfiguration("cabinet")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    tf_prefix = LaunchConfiguration("tf_prefix")

    moveit_config = (
        MoveItConfigsBuilder("hiwin_ra6")
        .robot_description(
            file_path="config/ra6.urdf.xacro",
            mappings={
                "name": "hiwin",
                "ra_type": ra_type,
                "use_fake_hardware": use_fake_hardware,
                "cabinet": cabinet,
                "robot_ip": robot_ip,
                "tf_prefix": tf_prefix,
            },
        )
        .robot_description_semantic(Path("config") / "ra6.srdf")
        .robot_description_kinematics(Path("config") / "kinematics.yaml")
        .trajectory_execution(Path("config") / "moveit_controllers.yaml")
        .to_moveit_configs()
    )

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hiwin_ra6_moveit_config"), "config", "ros2_controllers.yaml"]
    )

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

    # Spawn controllers
    def controller_spawner(controllers):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + controllers,
        )

    controllers_active = [
        "joint_state_broadcaster",
        "manipulator_controller",
    ]
    controller_spawners = [controller_spawner(controllers_active)]

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False},
        ],
    )

    # RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("hiwin_ra6_moveit_config"), "config", "moveit.rviz"]
    )
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

    nodes_to_start = [
        robot_state_publisher,
        control_node,
        move_group_node,
        rviz_node,
    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ra_type",
            description="Typo/series of used RA robot.",
            choices=[
                "ra605_710",
                "ra610_1355",
                "ra610_1869",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="0.0.0.0",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cabinet",
            default_value="gc2",
            description="Robot Control Cabinets from HIWIN.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated.",
        )
    )

    return LaunchDescription(declared_arguments + launch_setup())
