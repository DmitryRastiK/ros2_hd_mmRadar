import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy


class QoSBridgeNode(Node):
    def __init__(self):
        super().__init__("qos_bridge_node")

        qos_sub = QoSProfile(depth=10)
        qos_sub.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.listener_callback, qos_sub
        )

        qos_pub = QoSProfile(depth=10)
        qos_pub.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(LaserScan, "/scan_reliable", qos_pub)

        self.get_logger().info("QoS bridge node started")

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
