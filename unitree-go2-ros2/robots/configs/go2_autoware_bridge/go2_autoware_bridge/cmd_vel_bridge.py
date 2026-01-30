import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_cmd_vel_bridge")

        self.declare_parameter("input_twist_topic", "/control/command/twist")
        self.declare_parameter("input_type", "twist_stamped")
        self.declare_parameter("output_cmd_vel_topic", "/cmd_vel")

        input_twist_topic = (
            self.get_parameter("input_twist_topic").get_parameter_value().string_value
        )
        input_type = (
            self.get_parameter("input_type").get_parameter_value().string_value
        )
        output_cmd_vel_topic = (
            self.get_parameter("output_cmd_vel_topic").get_parameter_value().string_value
        )

        self._cmd_vel_pub = self.create_publisher(Twist, output_cmd_vel_topic, 10)

        if input_type == "twist":
            self.create_subscription(
                Twist,
                input_twist_topic,
                self._on_twist,
                10,
            )
        else:
            self.create_subscription(
                TwistStamped,
                input_twist_topic,
                self._on_twist_stamped,
                10,
            )

        self.get_logger().info(
            "Bridging control '%s' (%s) -> '%s'",
            input_twist_topic,
            input_type,
            output_cmd_vel_topic,
        )

    def _on_twist(self, msg: Twist) -> None:
        self._cmd_vel_pub.publish(msg)

    def _on_twist_stamped(self, msg: TwistStamped) -> None:
        self._cmd_vel_pub.publish(msg.twist)


def main() -> None:
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
