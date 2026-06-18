from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    yaw = LaunchConfiguration("yaw")
    gui = LaunchConfiguration("gui")

    package_share = FindPackageShare("virtual_indoor_nav")
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )
    robot_xacro = PathJoinSubstitution([package_share, "urdf", "indoor_bot.urdf.xacro"])
    ekf_config = PathJoinSubstitution([package_share, "config", "ekf.yaml"])

    robot_description = Command(["xacro ", robot_xacro])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "indoor_bot",
            "-topic",
            "robot_description",
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.08",
            "-Y",
            yaw,
        ],
    )

    ekf_filter = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [package_share, "worlds", "apartment.world"]
                ),
            ),
            DeclareLaunchArgument("x_pose", default_value="-7.0"),
            DeclareLaunchArgument("y_pose", default_value="2.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("gui", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "world": world,
                    "verbose": "true",
                    "gui": gui,
                }.items(),
            ),
            robot_state_publisher,
            spawn_robot,
            ekf_filter,
        ]
    )
