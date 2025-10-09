#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TrackerPathNode(Node):
    """
    ROS2 node that subscribes to radar dynamic markers and creates a path
    connecting the marker positions over time.
    """

    def __init__(self):
        super().__init__('tracker_path_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_topic', '/radar_dynamic_markers'),
                ('output_topic', '/tracker_path'),
                ('max_path_length', 1000),  # Maximum number of poses in the path
                ('target_frame', 'base_link'),  # Frame ID for the path
            ]
        )
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.max_path_length = self.get_parameter('max_path_length').value
        self.target_frame = self.get_parameter('target_frame').value
        
        # Initialize path
        self.path = Path()
        self.path.header.frame_id = self.target_frame
        
        # QoS profile for subscriber
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriber
        self.sub = self.create_subscription(
            MarkerArray,
            self.input_topic,
            self.marker_callback,
            marker_qos
        )
        
        # Create publisher for path
        self.pub = self.create_publisher(
            Path,
            self.output_topic,
            10
        )
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('Tracker Path Node initialized')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'  Input:  {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
        self.get_logger().info(f'  Max path length: {self.max_path_length}')
        self.get_logger().info(f'  Target frame: {self.target_frame}')
        self.get_logger().info('=' * 80)

    def marker_callback(self, msg):
        """
        Callback for MarkerArray messages.
        Extracts positions from markers and adds them to the path.
        """
        if len(msg.markers) == 0:
            return
        
        # Update path header with latest timestamp
        self.path.header.stamp = msg.markers[0].header.stamp
        self.path.header.frame_id = msg.markers[0].header.frame_id
        
        # Add each marker position to the path
        for marker in msg.markers:
            pose_stamped = PoseStamped()
            pose_stamped.header = marker.header
            pose_stamped.pose = marker.pose
            
            self.path.poses.append(pose_stamped)
        
        # Limit path length to avoid memory issues
        if len(self.path.poses) > self.max_path_length:
            # Remove oldest poses
            excess = len(self.path.poses) - self.max_path_length
            self.path.poses = self.path.poses[excess:]
        
        # Publish the path
        self.pub.publish(self.path)
        
        self.get_logger().debug(
            f'Path updated: {len(self.path.poses)} poses from {len(msg.markers)} markers'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackerPathNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

