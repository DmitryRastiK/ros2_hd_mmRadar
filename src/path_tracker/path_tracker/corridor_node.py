#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class CorridorNode(Node):
    """
    ROS2 node that publishes a corridor marker with configurable dimensions and position.
    The corridor is a rectangular box oriented along the x-axis.
    """
    
    def __init__(self):
        super().__init__('corridor_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Topic configuration
                ('output_topic', '/corridor_marker'),
                
                # Corridor dimensions (meters)
                ('length_x', 50.0),  # Length along x-axis
                ('width_y', 3.0),    # Width along y-axis
                ('height_z', 3.0),   # Height along z-axis
                
                # Corridor position (start point)
                ('start_x', 0.0),
                ('start_y', 0.0),
                ('start_z', 0.0),
                
                # Frame configuration
                ('frame_id', 'base_link'),
                
                # Marker appearance
                ('color_r', 0.0),
                ('color_g', 0.0),
                ('color_b', 1.0),
                ('color_a', 0.3),  # Semi-transparent
                
                # Publishing rate (Hz)
                ('publish_rate', 1.0),
                
                # Marker type: 'cube' or 'line_strip'
                ('marker_type', 'cube'),
            ]
        )
        
        # Get parameters
        self.output_topic = self.get_parameter('output_topic').value
        self.length_x = self.get_parameter('length_x').value
        self.width_y = self.get_parameter('width_y').value
        self.height_z = self.get_parameter('height_z').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_z = self.get_parameter('start_z').value
        self.frame_id = self.get_parameter('frame_id').value
        self.color_r = self.get_parameter('color_r').value
        self.color_g = self.get_parameter('color_g').value
        self.color_b = self.get_parameter('color_b').value
        self.color_a = self.get_parameter('color_a').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.marker_type_str = self.get_parameter('marker_type').value
        
        # Create publisher
        self.pub = self.create_publisher(
            Marker,
            self.output_topic,
            10
        )
        
        # Create timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_corridor)
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('Corridor Node initialized')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'  Output topic: {self.output_topic}')
        self.get_logger().info(f'  Dimensions: {self.length_x}m (L) x {self.width_y}m (W) x {self.height_z}m (H)')
        self.get_logger().info(f'  Start position: ({self.start_x}, {self.start_y}, {self.start_z})')
        self.get_logger().info(f'  Frame ID: {self.frame_id}')
        self.get_logger().info(f'  Color: RGBA({self.color_r}, {self.color_g}, {self.color_b}, {self.color_a})')
        self.get_logger().info(f'  Marker type: {self.marker_type_str}')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info('=' * 80)
    
    def publish_corridor(self):
        """Publish the corridor marker."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "corridor"
        marker.id = 0
        marker.action = Marker.ADD
        
        if self.marker_type_str == 'cube':
            # Publish as a single cube
            marker.type = Marker.CUBE
            
            # Position at the center of the corridor
            marker.pose.position.x = self.start_x + self.length_x / 2.0
            marker.pose.position.y = self.start_y
            marker.pose.position.z = self.start_z + self.height_z / 2.0
            marker.pose.orientation.w = 1.0
            
            # Scale defines the dimensions
            marker.scale.x = self.length_x
            marker.scale.y = self.width_y
            marker.scale.z = self.height_z
            
        elif self.marker_type_str == 'line_strip':
            # Publish as a wireframe box
            marker.type = Marker.LINE_STRIP
            
            # Line width
            marker.scale.x = 0.05
            
            # Define the 8 corners of the corridor box
            # Bottom face
            p0 = Point()
            p0.x = self.start_x
            p0.y = self.start_y - self.width_y / 2.0
            p0.z = self.start_z
            
            p1 = Point()
            p1.x = self.start_x + self.length_x
            p1.y = self.start_y - self.width_y / 2.0
            p1.z = self.start_z
            
            p2 = Point()
            p2.x = self.start_x + self.length_x
            p2.y = self.start_y + self.width_y / 2.0
            p2.z = self.start_z
            
            p3 = Point()
            p3.x = self.start_x
            p3.y = self.start_y + self.width_y / 2.0
            p3.z = self.start_z
            
            # Top face
            p4 = Point()
            p4.x = self.start_x
            p4.y = self.start_y - self.width_y / 2.0
            p4.z = self.start_z + self.height_z
            
            p5 = Point()
            p5.x = self.start_x + self.length_x
            p5.y = self.start_y - self.width_y / 2.0
            p5.z = self.start_z + self.height_z
            
            p6 = Point()
            p6.x = self.start_x + self.length_x
            p6.y = self.start_y + self.width_y / 2.0
            p6.z = self.start_z + self.height_z
            
            p7 = Point()
            p7.x = self.start_x
            p7.y = self.start_y + self.width_y / 2.0
            p7.z = self.start_z + self.height_z
            
            # Draw the wireframe
            # Bottom face
            marker.points.extend([p0, p1, p2, p3, p0])
            # Top face
            marker.points.extend([p4, p5, p6, p7, p4])
            # Vertical edges
            marker.points.extend([p0, p4])
            marker.points.append(p1)
            marker.points.append(p5)
            marker.points.append(p1)
            marker.points.append(p2)
            marker.points.append(p6)
            marker.points.append(p2)
            marker.points.append(p3)
            marker.points.append(p7)
        
        else:
            self.get_logger().warn(f'Unknown marker type: {self.marker_type_str}, using cube')
            marker.type = Marker.CUBE
            marker.pose.position.x = self.start_x + self.length_x / 2.0
            marker.pose.position.y = self.start_y
            marker.pose.position.z = self.start_z + self.height_z / 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.length_x
            marker.scale.y = self.width_y
            marker.scale.z = self.height_z
        
        # Color
        marker.color.r = self.color_r
        marker.color.g = self.color_g
        marker.color.b = self.color_b
        marker.color.a = self.color_a
        
        # Lifetime (0 = forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        # Publish
        self.pub.publish(marker)
        
        self.get_logger().debug('Published corridor marker')


def main(args=None):
    rclpy.init(args=args)
    node = CorridorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

