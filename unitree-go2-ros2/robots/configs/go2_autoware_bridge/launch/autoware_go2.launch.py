from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    enable_autoware = LaunchConfiguration("enable_autoware")
    autoware_launch_package = LaunchConfiguration("autoware_launch_package")
    autoware_launch_file = LaunchConfiguration("autoware_launch_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_autoware",
                default_value="false",
                description="If true, include Autoware Universe launch.",
            ),
            DeclareLaunchArgument(
                "autoware_launch_package",
                default_value="autoware_launch",
                description="Autoware Universe launch package name.",
            ),
            DeclareLaunchArgument(
                "autoware_launch_file",
                default_value="autoware.launch.xml",
                description="Autoware Universe launch file name.",
            ),
            Node(
                package="go2_autoware_bridge",
                executable="go2_autoware_bridge_node",
                name="go2_autoware_bridge",
                output="screen",
                parameters=[
                    {
                        "input_pointcloud_topic": "/velodyne_points",
                        "output_pointcloud_topic": "/sensing/lidar/top/pointcloud",
                        "pointcloud_frame_id": "velodyne",
                        "input_odom_topic": "/odom",
                        "output_odom_topic": "/localization/kinematic_state",
                        "odom_frame_id": "odom",
                        "base_frame_id": "base_link",
                    }
                ],
            ),
            Node(
                package="go2_autoware_bridge",
                executable="go2_cmd_vel_bridge_node",
                name="go2_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "input_twist_topic": "/control/command/twist",
                        "input_type": "twist_stamped",
                        "output_cmd_vel_topic": "/cmd_vel",
                    }
                ],
            ),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare(autoware_launch_package),
                                    "launch",
                                    autoware_launch_file,
                                ]
                            )
                        )
                    )
                ],
                condition=IfCondition(enable_autoware),
            ),
        ]
    )
