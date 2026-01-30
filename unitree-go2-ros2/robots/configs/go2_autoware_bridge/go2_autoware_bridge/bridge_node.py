from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class Go2AutowareBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_autoware_bridge")

        self.declare_parameter("input_pointcloud_topic", "/velodyne_points")
        self.declare_parameter(
            "output_pointcloud_topic", "/sensing/lidar/top/pointcloud"
        )
        self.declare_parameter("pointcloud_frame_id", "")

        self.declare_parameter("input_odom_topic", "/odom")
        self.declare_parameter(
            "output_odom_topic", "/localization/kinematic_state"
        )
        self.declare_parameter("odom_frame_id", "")
        self.declare_parameter("base_frame_id", "")

        input_pointcloud_topic = (
            self.get_parameter("input_pointcloud_topic").get_parameter_value().string_value
        )
        output_pointcloud_topic = (
            self.get_parameter("output_pointcloud_topic").get_parameter_value().string_value
        )
        self._pointcloud_frame_id = (
            self.get_parameter("pointcloud_frame_id").get_parameter_value().string_value
        )

        input_odom_topic = (
            self.get_parameter("input_odom_topic").get_parameter_value().string_value
        )
        output_odom_topic = (
            self.get_parameter("output_odom_topic").get_parameter_value().string_value
        )
        self._odom_frame_id = (
            self.get_parameter("odom_frame_id").get_parameter_value().string_value
        )
        self._base_frame_id = (
            self.get_parameter("base_frame_id").get_parameter_value().string_value
        )

        self._pointcloud_pub = self.create_publisher(PointCloud2, output_pointcloud_topic, 10)
        self._odom_pub = self.create_publisher(Odometry, output_odom_topic, 10)

        self.create_subscription(
            PointCloud2,
            input_pointcloud_topic,
            self._on_pointcloud,
            10,
        )
        self.create_subscription(
            Odometry,
            input_odom_topic,
            self._on_odom,
            10,
        )

        self.get_logger().info(
            "Bridging pointcloud '%s' -> '%s'",
            input_pointcloud_topic,
            output_pointcloud_topic,
        )
        self.get_logger().info(
            "Bridging odom '%s' -> '%s'",
            input_odom_topic,
            output_odom_topic,
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        if not self._pointcloud_frame_id:
            self._pointcloud_pub.publish(msg)
            return

        outgoing = deepcopy(msg)
        outgoing.header.frame_id = self._pointcloud_frame_id
        self._pointcloud_pub.publish(outgoing)

    def _on_odom(self, msg: Odometry) -> None:
        outgoing = deepcopy(msg)
        if self._odom_frame_id:
            outgoing.header.frame_id = self._odom_frame_id
        if self._base_frame_id:
            outgoing.child_frame_id = self._base_frame_id
        self._odom_pub.publish(outgoing)


def main() -> None:
    rclpy.init()
    node = Go2AutowareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
