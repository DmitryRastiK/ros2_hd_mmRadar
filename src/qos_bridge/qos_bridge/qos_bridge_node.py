import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy


class QoSBridgeNode(Node):
    def __init__(self):
        super().__init__("qos_bridge_node")

        # Declare configurable topic names
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_reliable")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        qos_sub = QoSProfile(depth=10)
        qos_sub.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.listener_callback, qos_sub
        )

        qos_pub = QoSProfile(depth=10)
        qos_pub.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(LaserScan, output_topic, qos_pub)

        self.get_logger().info(
            f"QoS bridge node started (input='{input_topic}', output='{output_topic}')"
        )

    def listener_callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QoSBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
