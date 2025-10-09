#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import csv
import os
from datetime import datetime


class DataLoggerNode(Node):
    """
    ROS2 node that logs radar point cloud data to CSV files.
    Creates separate CSV files for each parameter: rcs, velocity, x, y, z, snr.
    """
    
    def __init__(self):
        super().__init__('data_logger_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_topic', '/hd_radar_0/points/dynamic_filtered'),
                ('output_dir', 'analys'),
                ('file_prefix', 'radar_data'),
            ]
        )
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        
        # Create output directory if it doesn't exist
        if not os.path.isabs(self.output_dir):
            # If relative path, try to find workspace root
            # Look for workspace by checking for 'src' directory
            current_dir = os.getcwd()
            workspace_root = None
            
            # Check if we're in workspace (has 'src' subdirectory)
            if os.path.isdir(os.path.join(current_dir, 'src')):
                workspace_root = current_dir
            else:
                # Try parent directories
                parent = os.path.dirname(current_dir)
                while parent != '/':
                    if os.path.isdir(os.path.join(parent, 'src')):
                        workspace_root = parent
                        break
                    parent = os.path.dirname(parent)
            
            if workspace_root:
                # Save in workspace/src/path_tracker/analys
                self.output_dir = os.path.join(workspace_root, 'src', 'path_tracker', self.output_dir)
            else:
                # Fallback to current directory
                self.output_dir = os.path.abspath(self.output_dir)
        
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Generate unique timestamp for this session
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Create CSV file paths
        self.csv_files = {
            'rcs': os.path.join(self.output_dir, f'{self.file_prefix}_rcs_{self.session_timestamp}.csv'),
            'velocity': os.path.join(self.output_dir, f'{self.file_prefix}_velocity_{self.session_timestamp}.csv'),
            'x': os.path.join(self.output_dir, f'{self.file_prefix}_x_{self.session_timestamp}.csv'),
            'y': os.path.join(self.output_dir, f'{self.file_prefix}_y_{self.session_timestamp}.csv'),
            'z': os.path.join(self.output_dir, f'{self.file_prefix}_z_{self.session_timestamp}.csv'),
            'snr': os.path.join(self.output_dir, f'{self.file_prefix}_snr_{self.session_timestamp}.csv'),
        }
        
        # Open CSV files and write headers
        self.csv_file_handles = {}
        self.csv_writers = {}
        
        for param_name, file_path in self.csv_files.items():
            file_handle = open(file_path, 'w', newline='')
            writer = csv.writer(file_handle)
            writer.writerow(['timestamp', param_name])  # Header
            
            self.csv_file_handles[param_name] = file_handle
            self.csv_writers[param_name] = writer
        
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
        
        # Statistics
        self.total_points_logged = 0
        self.messages_received = 0
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('Data Logger Node initialized')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'  Input topic: {self.input_topic}')
        self.get_logger().info(f'  Output directory: {self.output_dir}')
        self.get_logger().info(f'  Session: {self.session_timestamp}')
        self.get_logger().info('-' * 80)
        self.get_logger().info('CSV files created:')
        for param_name, file_path in self.csv_files.items():
            self.get_logger().info(f'  {param_name}: {os.path.basename(file_path)}')
        self.get_logger().info('=' * 80)
    
    def cloud_callback(self, msg):
        """Callback for point cloud messages."""
        self.messages_received += 1
        
        # Get timestamp from message header
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Get available field names
        field_names = [field.name for field in msg.fields]
        
        # Read points
        points = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))
        
        if len(points) == 0:
            return
        
        # Write each point to CSV files
        for point in points:
            point_dict = dict(zip(field_names, point))
            
            # Write to RCS file
            if 'rcs' in point_dict:
                self.csv_writers['rcs'].writerow([timestamp, point_dict['rcs']])
            
            # Write to velocity file
            if 'velocity' in point_dict:
                self.csv_writers['velocity'].writerow([timestamp, point_dict['velocity']])
            
            # Write to X file
            if 'x' in point_dict:
                self.csv_writers['x'].writerow([timestamp, point_dict['x']])
            
            # Write to Y file
            if 'y' in point_dict:
                self.csv_writers['y'].writerow([timestamp, point_dict['y']])
            
            # Write to Z file
            if 'z' in point_dict:
                self.csv_writers['z'].writerow([timestamp, point_dict['z']])
            
            # Write to SNR file
            if 'snr' in point_dict:
                self.csv_writers['snr'].writerow([timestamp, point_dict['snr']])
            
            self.total_points_logged += 1
        
        # Flush files periodically to ensure data is written
        if self.messages_received % 10 == 0:
            for file_handle in self.csv_file_handles.values():
                file_handle.flush()
        
        self.get_logger().debug(
            f'Logged {len(points)} points | Total: {self.total_points_logged} points from {self.messages_received} messages'
        )
    
    def __del__(self):
        """Cleanup: close all CSV files."""
        for file_handle in self.csv_file_handles.values():
            file_handle.close()


def main(args=None):
    rclpy.init(args=args)
    node = DataLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Log final statistics
        node.get_logger().info('=' * 80)
        node.get_logger().info('Data Logger shutdown')
        node.get_logger().info('=' * 80)
        node.get_logger().info(f'Total points logged: {node.total_points_logged}')
        node.get_logger().info(f'Total messages processed: {node.messages_received}')
        node.get_logger().info(f'Files saved in: {node.output_dir}')
        node.get_logger().info('=' * 80)
        
        # Close all files
        for file_handle in node.csv_file_handles.values():
            file_handle.close()
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

