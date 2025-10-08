#!/usr/bin/env python3
"""
ROS2 Node for filtering radar point clouds based on various parameters.
Subscribes to radar point cloud topics and publishes filtered point clouds.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointField
import struct
import numpy as np


class RadarPointCloudFilter(Node):
    def __init__(self):
        super().__init__('radar_point_cloud_filter')
        
        # Declare and get parameters for topics
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_topic', '/hd_radar_0/points/static'),
                ('output_topic', '/hd_radar_0/points/static_f'),
            ]
        )
        
        # Declare and get filter parameters with min and max values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('filter.x.enable', True),
                ('filter.x.min', -100.0),
                ('filter.x.max', 100.0),
                
                ('filter.y.enable', True),
                ('filter.y.min', -100.0),
                ('filter.y.max', 100.0),
                
                ('filter.z.enable', True),
                ('filter.z.min', -10.0),
                ('filter.z.max', 10.0),
                
                ('filter.v.enable', False),
                ('filter.v.min_abs', 0.5),  # Minimum absolute velocity (m/s)
                
                ('filter.self_v.enable', False),
                ('filter.self_v.min', -50.0),
                ('filter.self_v.max', 50.0),
                
                ('filter.snr.enable', True),
                ('filter.snr.min', 10.0),
                ('filter.snr.max', 100.0),
                
                ('filter.rcs.enable', True),
                ('filter.rcs.min', -50.0),
                ('filter.rcs.max', 50.0),
            ]
        )
        
        # Get topic names
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Load filter settings
        self.load_filter_settings()
        
        # QoS profile matching the radar driver (SensorDataQoS with BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriber with matching QoS
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            sensor_qos
        )
        
        # Create publisher with same QoS
        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            sensor_qos
        )
        
        # Statistics counters
        self.received = 0
        self.published = 0
        self.total_points_received = 0
        self.total_points_published = 0
        
        # Create timer for status output (every 5 seconds)
        self.timer = self.create_timer(5.0, self.print_statistics)
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('Radar Point Cloud Filter Node initialized')
        self.get_logger().info('=' * 80)
        self.get_logger().info('Topics configuration:')
        self.get_logger().info(f'  Input:  {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
        self.get_logger().info('-' * 80)
        self.log_filter_settings()
        self.get_logger().info('=' * 80)

    def load_filter_settings(self):
        """Load filter settings from parameters."""
        self.filters = {}
        
        # Load regular range filters (min-max)
        range_params = ['x', 'y', 'z', 'self_v', 'snr', 'rcs']
        for param in range_params:
            self.filters[param] = {
                'enable': self.get_parameter(f'filter.{param}.enable').value,
                'min': self.get_parameter(f'filter.{param}.min').value,
                'max': self.get_parameter(f'filter.{param}.max').value,
                'type': 'range'
            }
        
        # Load velocity filter (absolute minimum)
        self.filters['v'] = {
            'enable': self.get_parameter('filter.v.enable').value,
            'min_abs': self.get_parameter('filter.v.min_abs').value,
            'type': 'abs'
        }

    def log_filter_settings(self):
        """Log current filter settings."""
        self.get_logger().info('Filter settings:')
        for param, settings in self.filters.items():
            if settings['enable']:
                if settings['type'] == 'range':
                    self.get_logger().info(
                        f'  {param:7s}: [{settings["min"]:8.2f}, {settings["max"]:8.2f}]'
                    )
                elif settings['type'] == 'abs':
                    self.get_logger().info(
                        f'  {param:7s}: |v| >= {settings["min_abs"]:.2f} m/s'
                    )
            else:
                self.get_logger().info(f'  {param:7s}: DISABLED')

    def cloud_callback(self, msg):
        """Callback for point cloud."""
        self.received += 1
        self.total_points_received += msg.width
        
        filtered_msg = self.filter_point_cloud(msg)
        if filtered_msg is not None:
            self.published += 1
            self.total_points_published += filtered_msg.width
            self.pub.publish(filtered_msg)

    def filter_point_cloud(self, cloud_msg):
        
        # Read points from PointCloud2
        points_list = []
        
        # Parse point cloud data
        for point in pc2.read_points(cloud_msg, field_names=('x', 'y', 'z', 'v', 'self_v', 'snr', 'rcs'), skip_nans=True):
            x, y, z, v, self_v, snr, rcs = point
            
            # Apply filters
            if self.check_point_passes_filters(x, y, z, v, self_v, snr, rcs):
                points_list.append([x, y, z, v, self_v, snr, rcs])
        
        # If no points pass the filter, return None
        if len(points_list) == 0:
            return None
        
        # Create new PointCloud2 message
        filtered_msg = self.create_point_cloud2(cloud_msg.header, points_list)
        
        return filtered_msg

    def check_point_passes_filters(self, x, y, z, v, self_v, snr, rcs):
        
        values = {
            'x': x,
            'y': y,
            'z': z,
            'v': v,
            'self_v': self_v,
            'snr': snr,
            'rcs': rcs
        }
        
        for param, value in values.items():
            filter_settings = self.filters[param]
            if filter_settings['enable']:
                # For velocity, check absolute value
                if filter_settings['type'] == 'abs':
                    if abs(value) < filter_settings['min_abs']:
                        return False
                # For other parameters, check range
                elif filter_settings['type'] == 'range':
                    if value < filter_settings['min'] or value > filter_settings['max']:
                        return False
        
        return True

    def create_point_cloud2(self, header, points):
        
        # Define fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='v', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='self_v', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='snr', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='rcs', offset=24, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 28  # 7 fields * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.fields = fields
        
        # Pack point data
        buffer = []
        for point in points:
            for value in point:
                buffer.extend(struct.pack('f', value))
        
        msg.data = buffer
        
        return msg

    def print_statistics(self):
        """Print filtering statistics."""
        messages_filtered = self.received - self.published
        points_filtered = self.total_points_received - self.total_points_published
        
        if self.received > 0:
            msg_percent = (self.published / self.received) * 100
        else:
            msg_percent = 0.0
            
        if self.total_points_received > 0:
            points_percent = (self.total_points_published / self.total_points_received) * 100
        else:
            points_percent = 0.0
        
        self.get_logger().info(
            f'Messages: {self.received} recv, {self.published} pub, {messages_filtered} filtered ({msg_percent:.1f}% passed) | '
            f'Points: {self.total_points_received} recv, {self.total_points_published} pub, {points_filtered} filtered ({points_percent:.1f}% passed)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RadarPointCloudFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
