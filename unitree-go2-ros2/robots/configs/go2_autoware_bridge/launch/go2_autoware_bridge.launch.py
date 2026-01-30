from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
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
            )
        ]
    )
