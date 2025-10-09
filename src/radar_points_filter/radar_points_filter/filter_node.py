#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from hd_radar_interfaces.srv import SetRadarPointsFilter
import struct
import math


class RadarPointsFilterNode(Node):
    """
    ROS2 node that filters radar point cloud data based on various parameters.
    Supports filtering by range, RCS, velocity, azimuth, elevation, height, and SNR.
    """
    
    def __init__(self):
        super().__init__('filter_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Topics configuration
                ('input_topic', '/hd_radar_0/points/dynamic'),
                ('output_topic', '/hd_radar_0/points/dynamic_filtered'),
                
                # Range filter (meters)
                ('enable_range_filter', True),
                ('range_min', 0.0),
                ('range_max', 100.0),
                
                # RCS filter (dBsm - decibel square meters)
                ('enable_rcs_filter', True),
                ('rcs_min', -50.0),
                ('rcs_max', 50.0),
                
                # Velocity filter (m/s)
                ('enable_velocity_filter', False),
                ('velocity_abs_threshold', 0.5),  # Threshold for absolute velocity value
                ('velocity_filter_more', True),  # True: keep |v| > threshold, False: keep |v| < threshold
                
                # Azimuth filter (degrees)
                ('enable_azimuth_filter', False),
                ('azimuth_min', -180.0),
                ('azimuth_max', 180.0),
                
                # Elevation filter (degrees)
                ('enable_elevation_filter', False),
                ('elevation_min', -90.0),
                ('elevation_max', 90.0),
                
                # Height filter (z coordinate in meters)
                ('enable_height_filter', False),
                ('height_min', -10.0),
                ('height_max', 10.0),
                
                # X coordinate filter (meters)
                ('enable_x_filter', False),
                ('x_min', -100.0),
                ('x_max', 100.0),
                
                # Y coordinate filter (meters)
                ('enable_y_filter', False),
                ('y_min', -100.0),
                ('y_max', 100.0),
                
                # SNR filter (signal-to-noise ratio in dB)
                ('enable_snr_filter', False),
                ('snr_min', 0.0),
                ('snr_max', 100.0),
                
                # Power filter (dB)
                ('enable_power_filter', False),
                ('power_min', -100.0),
                ('power_max', 100.0),
                
                # Noise filter (dB)
                ('enable_noise_filter', False),
                ('noise_min', -100.0),
                ('noise_max', 100.0),
            ]
        )
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # Range filter
        self.enable_range_filter = self.get_parameter('enable_range_filter').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        
        # RCS filter
        self.enable_rcs_filter = self.get_parameter('enable_rcs_filter').value
        self.rcs_min = self.get_parameter('rcs_min').value
        self.rcs_max = self.get_parameter('rcs_max').value
        
        # Velocity filter
        self.enable_velocity_filter = self.get_parameter('enable_velocity_filter').value
        self.velocity_abs_threshold = self.get_parameter('velocity_abs_threshold').value
        self.velocity_filter_more = self.get_parameter('velocity_filter_more').value
        
        # Azimuth filter
        self.enable_azimuth_filter = self.get_parameter('enable_azimuth_filter').value
        self.azimuth_min = self.get_parameter('azimuth_min').value
        self.azimuth_max = self.get_parameter('azimuth_max').value
        
        # Elevation filter
        self.enable_elevation_filter = self.get_parameter('enable_elevation_filter').value
        self.elevation_min = self.get_parameter('elevation_min').value
        self.elevation_max = self.get_parameter('elevation_max').value
        
        # Height filter
        self.enable_height_filter = self.get_parameter('enable_height_filter').value
        self.height_min = self.get_parameter('height_min').value
        self.height_max = self.get_parameter('height_max').value
        
        # X filter
        self.enable_x_filter = self.get_parameter('enable_x_filter').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        
        # Y filter
        self.enable_y_filter = self.get_parameter('enable_y_filter').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        
        # SNR filter
        self.enable_snr_filter = self.get_parameter('enable_snr_filter').value
        self.snr_min = self.get_parameter('snr_min').value
        self.snr_max = self.get_parameter('snr_max').value
        
        # Power filter
        self.enable_power_filter = self.get_parameter('enable_power_filter').value
        self.power_min = self.get_parameter('power_min').value
        self.power_max = self.get_parameter('power_max').value
        
        # Noise filter
        self.enable_noise_filter = self.get_parameter('enable_noise_filter').value
        self.noise_min = self.get_parameter('noise_min').value
        self.noise_max = self.get_parameter('noise_max').value
        
        # QoS profile matching the radar driver
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriber
        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.cloud_callback,
            sensor_qos
        )
        
        # Create publisher
        self.pub = self.create_publisher(
            PointCloud2,
            self.output_topic,
            sensor_qos
        )
        
        # Statistics
        self.total_input_points = 0
        self.total_output_points = 0
        self.message_count = 0
        
        # Statistics reporting interval
        self.declare_parameter('stats_report_interval', 100)  # Report every N messages
        self.stats_report_interval = self.get_parameter('stats_report_interval').value
        
        # Add parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Create service for setting filter parameters
        self.srv_set_filter = self.create_service(
            SetRadarPointsFilter,
            '~/set_filter',
            self.set_filter_callback
        )
        
        self.log_initialization()
    
    def parameters_callback(self, params):
        """
        Callback for dynamic parameter changes.
        Allows runtime reconfiguration of filter parameters.
        """
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            param_name = param.name
            param_value = param.value
            
            # Update filter parameters dynamically
            if param_name == 'enable_range_filter':
                self.enable_range_filter = param_value
            elif param_name == 'range_min':
                self.range_min = param_value
            elif param_name == 'range_max':
                self.range_max = param_value
            elif param_name == 'enable_rcs_filter':
                self.enable_rcs_filter = param_value
            elif param_name == 'rcs_min':
                self.rcs_min = param_value
            elif param_name == 'rcs_max':
                self.rcs_max = param_value
            elif param_name == 'enable_velocity_filter':
                self.enable_velocity_filter = param_value
            elif param_name == 'velocity_abs_threshold':
                self.velocity_abs_threshold = param_value
            elif param_name == 'velocity_filter_more':
                self.velocity_filter_more = param_value
            elif param_name == 'enable_azimuth_filter':
                self.enable_azimuth_filter = param_value
            elif param_name == 'azimuth_min':
                self.azimuth_min = param_value
            elif param_name == 'azimuth_max':
                self.azimuth_max = param_value
            elif param_name == 'enable_elevation_filter':
                self.enable_elevation_filter = param_value
            elif param_name == 'elevation_min':
                self.elevation_min = param_value
            elif param_name == 'elevation_max':
                self.elevation_max = param_value
            elif param_name == 'enable_height_filter':
                self.enable_height_filter = param_value
            elif param_name == 'height_min':
                self.height_min = param_value
            elif param_name == 'height_max':
                self.height_max = param_value
            elif param_name == 'enable_x_filter':
                self.enable_x_filter = param_value
            elif param_name == 'x_min':
                self.x_min = param_value
            elif param_name == 'x_max':
                self.x_max = param_value
            elif param_name == 'enable_y_filter':
                self.enable_y_filter = param_value
            elif param_name == 'y_min':
                self.y_min = param_value
            elif param_name == 'y_max':
                self.y_max = param_value
            elif param_name == 'enable_snr_filter':
                self.enable_snr_filter = param_value
            elif param_name == 'snr_min':
                self.snr_min = param_value
            elif param_name == 'snr_max':
                self.snr_max = param_value
            elif param_name == 'enable_power_filter':
                self.enable_power_filter = param_value
            elif param_name == 'power_min':
                self.power_min = param_value
            elif param_name == 'power_max':
                self.power_max = param_value
            elif param_name == 'enable_noise_filter':
                self.enable_noise_filter = param_value
            elif param_name == 'noise_min':
                self.noise_min = param_value
            elif param_name == 'noise_max':
                self.noise_max = param_value
            elif param_name == 'stats_report_interval':
                self.stats_report_interval = param_value
            
            self.get_logger().info(f'Parameter updated: {param_name} = {param_value}')
        
        return SetParametersResult(successful=True)
    
    def set_filter_callback(self, request, response):
        """
        Service callback for setting filter parameters.
        Allows setting multiple parameters at once.
        """
        try:
            # Update range filter
            self.enable_range_filter = request.enable_range_filter
            self.range_min = request.range_min
            self.range_max = request.range_max
            
            # Update RCS filter
            self.enable_rcs_filter = request.enable_rcs_filter
            self.rcs_min = request.rcs_min
            self.rcs_max = request.rcs_max
            
            # Update velocity filter
            self.enable_velocity_filter = request.enable_velocity_filter
            self.velocity_abs_threshold = request.velocity_abs_threshold
            self.velocity_filter_more = request.velocity_filter_more
            
            # Update azimuth filter
            self.enable_azimuth_filter = request.enable_azimuth_filter
            self.azimuth_min = request.azimuth_min
            self.azimuth_max = request.azimuth_max
            
            # Update elevation filter
            self.enable_elevation_filter = request.enable_elevation_filter
            self.elevation_min = request.elevation_min
            self.elevation_max = request.elevation_max
            
            # Update height filter
            self.enable_height_filter = request.enable_height_filter
            self.height_min = request.height_min
            self.height_max = request.height_max
            
            # Update X filter
            self.enable_x_filter = request.enable_x_filter
            self.x_min = request.x_min
            self.x_max = request.x_max
            
            # Update Y filter
            self.enable_y_filter = request.enable_y_filter
            self.y_min = request.y_min
            self.y_max = request.y_max
            
            # Update SNR filter
            self.enable_snr_filter = request.enable_snr_filter
            self.snr_min = request.snr_min
            self.snr_max = request.snr_max
            
            # Update power filter
            self.enable_power_filter = request.enable_power_filter
            self.power_min = request.power_min
            self.power_max = request.power_max
            
            # Update noise filter
            self.enable_noise_filter = request.enable_noise_filter
            self.noise_min = request.noise_min
            self.noise_max = request.noise_max
            
            response.success = True
            response.message = "Filter parameters updated successfully"
            self.get_logger().info('Filter parameters updated via service')
            
            # Log active filters
            active_filters = []
            if self.enable_range_filter:
                active_filters.append(f'Range: [{self.range_min:.2f}, {self.range_max:.2f}]m')
            if self.enable_rcs_filter:
                active_filters.append(f'RCS: [{self.rcs_min:.2f}, {self.rcs_max:.2f}]dBsm')
            if self.enable_velocity_filter:
                op = '>' if self.velocity_filter_more else '<'
                active_filters.append(f'Velocity: |v|{op}{self.velocity_abs_threshold:.2f}m/s')
            if self.enable_height_filter:
                active_filters.append(f'Height: [{self.height_min:.2f}, {self.height_max:.2f}]m')
            
            if active_filters:
                self.get_logger().info(f'Active filters: {", ".join(active_filters)}')
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to update filter parameters: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def log_initialization(self):
        """Log initialization information."""
        self.get_logger().info('=' * 80)
        self.get_logger().info('Radar Points Filter Node initialized')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'  Input:  {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
        self.get_logger().info('-' * 80)
        self.get_logger().info('Active filters:')
        
        if self.enable_range_filter:
            self.get_logger().info(f'  Range: [{self.range_min:.2f}, {self.range_max:.2f}] m')
        
        if self.enable_rcs_filter:
            self.get_logger().info(f'  RCS: [{self.rcs_min:.2f}, {self.rcs_max:.2f}] dBsm')
        
        if self.enable_velocity_filter:
            filter_type = ">" if self.velocity_filter_more else "<"
            self.get_logger().info(
                f'  Velocity: |v| {filter_type} {self.velocity_abs_threshold:.2f} m/s'
            )
        
        if self.enable_azimuth_filter:
            self.get_logger().info(f'  Azimuth: [{self.azimuth_min:.2f}, {self.azimuth_max:.2f}] deg')
        
        if self.enable_elevation_filter:
            self.get_logger().info(f'  Elevation: [{self.elevation_min:.2f}, {self.elevation_max:.2f}] deg')
        
        if self.enable_height_filter:
            self.get_logger().info(f'  Height (Z): [{self.height_min:.2f}, {self.height_max:.2f}] m')
        
        if self.enable_x_filter:
            self.get_logger().info(f'  X: [{self.x_min:.2f}, {self.x_max:.2f}] m')
        
        if self.enable_y_filter:
            self.get_logger().info(f'  Y: [{self.y_min:.2f}, {self.y_max:.2f}] m')
        
        if self.enable_snr_filter:
            self.get_logger().info(f'  SNR: [{self.snr_min:.2f}, {self.snr_max:.2f}] dB')
        
        if self.enable_power_filter:
            self.get_logger().info(f'  Power: [{self.power_min:.2f}, {self.power_max:.2f}] dB')
        
        if self.enable_noise_filter:
            self.get_logger().info(f'  Noise: [{self.noise_min:.2f}, {self.noise_max:.2f}] dB')
        
        self.get_logger().info('-' * 80)
        self.get_logger().info('Dynamic reconfiguration enabled!')
        self.get_logger().info('  Method 1 - Parameters: ros2 param set /filter_node <param> <value>')
        self.get_logger().info('  Method 2 - Service: ros2 service call /filter_node/set_filter ...')
        self.get_logger().info('=' * 80)
    
    def apply_filters(self, point_data):
        """
        Apply all enabled filters to a point.
        
        Args:
            point_data: Dictionary with point fields
        
        Returns:
            bool: True if point passes all filters, False otherwise
        """
        x = point_data.get('x', 0.0)
        y = point_data.get('y', 0.0)
        z = point_data.get('z', 0.0)
        
        # Range filter
        if self.enable_range_filter:
            range_val = math.sqrt(x*x + y*y + z*z)
            if range_val < self.range_min or range_val > self.range_max:
                return False
        
        # RCS filter
        if self.enable_rcs_filter:
            rcs = point_data.get('rcs', 0.0)
            if rcs < self.rcs_min or rcs > self.rcs_max:
                return False
        
        # Velocity filter (by absolute value)
        if self.enable_velocity_filter:
            velocity = point_data.get('velocity', 0.0)
            velocity_abs = abs(velocity)
            if self.velocity_filter_more:
                # Keep points with |v| > threshold
                if velocity_abs <= self.velocity_abs_threshold:
                    return False
            else:
                # Keep points with |v| < threshold
                if velocity_abs >= self.velocity_abs_threshold:
                    return False
        
        # Azimuth filter
        if self.enable_azimuth_filter:
            azimuth = point_data.get('azimuth', 0.0)
            if azimuth < self.azimuth_min or azimuth > self.azimuth_max:
                return False
        
        # Elevation filter
        if self.enable_elevation_filter:
            elevation = point_data.get('elevation', 0.0)
            if elevation < self.elevation_min or elevation > self.elevation_max:
                return False
        
        # Height (Z) filter
        if self.enable_height_filter:
            if z < self.height_min or z > self.height_max:
                return False
        
        # X filter
        if self.enable_x_filter:
            if x < self.x_min or x > self.x_max:
                return False
        
        # Y filter
        if self.enable_y_filter:
            if y < self.y_min or y > self.y_max:
                return False
        
        # SNR filter
        if self.enable_snr_filter:
            snr = point_data.get('snr', 0.0)
            if snr < self.snr_min or snr > self.snr_max:
                return False
        
        # Power filter
        if self.enable_power_filter:
            power = point_data.get('power', 0.0)
            if power < self.power_min or power > self.power_max:
                return False
        
        # Noise filter
        if self.enable_noise_filter:
            noise = point_data.get('noise', 0.0)
            if noise < self.noise_min or noise > self.noise_max:
                return False
        
        return True
    
    def cloud_callback(self, msg):
        """Callback for point cloud messages."""
        # Get available field names
        field_names = [field.name for field in msg.fields]
        
        # Read points
        points_in = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))
        input_count = len(points_in)
        self.total_input_points += input_count
        
        # Filter points
        filtered_points = []
        for point in points_in:
            # Create dictionary from point tuple
            point_dict = dict(zip(field_names, point))
            
            if self.apply_filters(point_dict):
                filtered_points.append(point)
        
        output_count = len(filtered_points)
        filtered_count = input_count - output_count
        self.total_output_points += output_count
        self.message_count += 1
        
        # Create output point cloud
        if len(filtered_points) > 0:
            output_msg = pc2.create_cloud(msg.header, msg.fields, filtered_points)
            self.pub.publish(output_msg)
        else:
            # Publish empty cloud
            output_msg = PointCloud2()
            output_msg.header = msg.header
            output_msg.fields = msg.fields
            output_msg.height = 1
            output_msg.width = 0
            output_msg.is_bigendian = msg.is_bigendian
            output_msg.point_step = msg.point_step
            output_msg.row_step = 0
            output_msg.is_dense = msg.is_dense
            output_msg.data = b''
            self.pub.publish(output_msg)
        
        # Log per-message statistics
        pass_rate = (output_count / input_count * 100) if input_count > 0 else 0
        self.get_logger().info(
            f'Points: {input_count} input | {output_count} passed ({pass_rate:.1f}%) | '
            f'{filtered_count} filtered ({100-pass_rate:.1f}%)'
        )
        
        # Periodically log cumulative statistics
        if self.message_count % self.stats_report_interval == 0:
            total_filtered = self.total_input_points - self.total_output_points
            total_pass_rate = (self.total_output_points / self.total_input_points * 100) \
                if self.total_input_points > 0 else 0
            self.get_logger().info('=' * 80)
            self.get_logger().info(
                f'Cumulative Statistics (after {self.message_count} messages):'
            )
            self.get_logger().info(
                f'  Total input:    {self.total_input_points} points'
            )
            self.get_logger().info(
                f'  Total passed:   {self.total_output_points} points ({total_pass_rate:.1f}%)'
            )
            self.get_logger().info(
                f'  Total filtered: {total_filtered} points ({100-total_pass_rate:.1f}%)'
            )
            self.get_logger().info('=' * 80)


def main(args=None):
    rclpy.init(args=args)
    node = RadarPointsFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Log final statistics
        if node.total_input_points > 0:
            total_filtered = node.total_input_points - node.total_output_points
            total_pass_rate = node.total_output_points / node.total_input_points * 100
            node.get_logger().info('=' * 80)
            node.get_logger().info('FINAL STATISTICS')
            node.get_logger().info('=' * 80)
            node.get_logger().info(f'  Total messages processed: {node.message_count}')
            node.get_logger().info(f'  Total input points:       {node.total_input_points}')
            node.get_logger().info(f'  Total passed points:      {node.total_output_points} ({total_pass_rate:.1f}%)')
            node.get_logger().info(f'  Total filtered points:    {total_filtered} ({100-total_pass_rate:.1f}%)')
            node.get_logger().info('=' * 80)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

