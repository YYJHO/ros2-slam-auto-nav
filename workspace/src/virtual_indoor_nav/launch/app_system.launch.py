"""
app_system.launch.py

Unified launch file that starts the entire system in one ROS session:
  - Gazebo simulation world + robot
  - slam_toolbox (online async SLAM)
  - Nav2 bringup (live SLAM map, no AMCL)
  - auto_explorer (frontier exploration with coverage tracking)
  - room_nav_node (room save/goto/rename/auto-detect)
  - Optional RViz

Once this launch is running, control_center.py only sends ROS commands
(/room_command) to switch between exploration/save/navigation.
No Gazebo restart is needed for navigation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    min_coverage_for_done = LaunchConfiguration("min_coverage_for_done")
    verification_required_clear_rounds = LaunchConfiguration("verification_required_clear_rounds")

    package_share = FindPackageShare("virtual_indoor_nav")

    sim_launch = PathJoinSubstitution([package_share, "launch", "sim_world.launch.py"])
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )
    slam_params = PathJoinSubstitution([package_share, "config", "slam_toolbox.yaml"])
    nav2_live_launch = PathJoinSubstitution(
        [package_share, "launch", "nav2_live.launch.py"]
    )
    nav2_params = PathJoinSubstitution([package_share, "config", "nav2_params.yaml"])
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
                "min_coverage_for_done": min_coverage_for_done,
                "verification_required_clear_rounds": verification_required_clear_rounds,
            },
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
            DeclareLaunchArgument("min_coverage_for_done", default_value="0.85"),
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

            # Nav2 navigation-only (no AMCL, no map_server — uses live SLAM /map)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_live_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "autostart": "true",
                }.items(),
            ),

            TimerAction(
                period=3.0,
                actions=[auto_explorer],
            ),

            TimerAction(
                period=4.0,
                actions=[room_nav],
            ),

            rviz_node,
        ]
    )
