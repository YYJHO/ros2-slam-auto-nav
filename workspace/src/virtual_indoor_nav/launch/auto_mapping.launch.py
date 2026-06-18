from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    exploration_timeout = LaunchConfiguration("exploration_timeout")
    initial_spin_duration = LaunchConfiguration("initial_spin_duration")
    no_frontier_streak_limit = LaunchConfiguration("no_frontier_streak_limit")
    frontier_refresh_interval = LaunchConfiguration("frontier_refresh_interval")
    recovery_duration = LaunchConfiguration("recovery_duration")
    laser_max_range = LaunchConfiguration("laser_max_range")
    min_coverage_for_done = LaunchConfiguration("min_coverage_for_done")
    stagnation_coverage_threshold = LaunchConfiguration("stagnation_coverage_threshold")
    verification_required_clear_rounds = LaunchConfiguration("verification_required_clear_rounds")

    package_share = FindPackageShare("virtual_indoor_nav")
    sim_launch = PathJoinSubstitution([package_share, "launch", "sim_world.launch.py"])
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )
    slam_params = PathJoinSubstitution([package_share, "config", "slam_toolbox.yaml"])
    exploration_params = PathJoinSubstitution(
        [package_share, "config", "exploration_params.yaml"]
    )
    rviz_config = PathJoinSubstitution([package_share, "rviz", "default.rviz"])

    auto_explorer = Node(
        package="virtual_indoor_nav",
        executable="auto_explorer",
        name="auto_explorer",
        output="screen",
        parameters=[
            exploration_params,
            {
                "use_sim_time": use_sim_time,
                "max_exploration_time": exploration_timeout,
                "initial_spin_duration": initial_spin_duration,
                "no_frontier_streak_limit": no_frontier_streak_limit,
                "frontier_refresh_interval": frontier_refresh_interval,
                "recovery_duration": recovery_duration,
                "laser_max_range": laser_max_range,
                "min_coverage_for_done": min_coverage_for_done,
                "stagnation_coverage_threshold": stagnation_coverage_threshold,
                "verification_required_clear_rounds": verification_required_clear_rounds,
            }
        ],
    )

    room_nav = Node(
        package="virtual_indoor_nav",
        executable="room_nav_node",
        name="room_nav_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("exploration_timeout", default_value="1200.0"),
            DeclareLaunchArgument("initial_spin_duration", default_value="10.0"),
            DeclareLaunchArgument("no_frontier_streak_limit", default_value="15"),
            DeclareLaunchArgument("frontier_refresh_interval", default_value="2.0"),
            DeclareLaunchArgument("recovery_duration", default_value="2.0"),
            DeclareLaunchArgument("laser_max_range", default_value="12.0"),
            DeclareLaunchArgument("min_coverage_for_done", default_value="0.85"),
            DeclareLaunchArgument("stagnation_coverage_threshold", default_value="0.03"),
            DeclareLaunchArgument("verification_required_clear_rounds", default_value="3"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "gui": gui,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": slam_params,
                }.items(),
            ),
            auto_explorer,
            room_nav,
            rviz_node,
        ]
    )
