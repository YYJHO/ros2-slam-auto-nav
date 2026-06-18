from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    rooms_file = LaunchConfiguration("rooms_file")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    project_root = Path(__file__).resolve().parents[4]
    default_map = str(project_root / "runtime" / "maps" / "generated_map.yaml")
    default_rooms = str(project_root / "runtime" / "rooms.yaml")

    package_share = FindPackageShare("virtual_indoor_nav")
    sim_launch = PathJoinSubstitution([package_share, "launch", "sim_world.launch.py"])
    nav2_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )
    nav2_params = PathJoinSubstitution([package_share, "config", "nav2_params.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "rviz", "default.rviz"])

    room_nav = Node(
        package="virtual_indoor_nav",
        executable="room_nav_node",
        name="room_nav_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "rooms_file": rooms_file,
            }
        ],
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
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
            ),
            DeclareLaunchArgument(
                "rooms_file",
                default_value=default_rooms,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "gui": gui,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_file,
                    "params_file": nav2_params,
                    "autostart": "true",
                    "use_composition": TextSubstitution(text="False"),
                }.items(),
            ),
            room_nav,
            rviz_node,
        ]
    )
