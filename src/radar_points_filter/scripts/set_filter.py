#!/usr/bin/env python3
"""
Convenient script to call the radar points filter service.
Allows easy configuration of filter parameters.
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
from hd_radar_interfaces.srv import SetRadarPointsFilter


def main():
    parser = argparse.ArgumentParser(
        description='Set radar points filter parameters via service',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Enable velocity filter with threshold 0.3 m/s (keep fast objects)
  python3 set_filter.py --velocity-enable true --velocity-threshold 0.3 --velocity-more true
  
  # Set range filter from 1 to 50 meters
  python3 set_filter.py --range-enable true --range-min 1.0 --range-max 50.0
  
  # Set RCS filter
  python3 set_filter.py --rcs-enable true --rcs-min -15.0 --rcs-max 5.0
  
  # Disable all filters
  python3 set_filter.py --range-enable false --rcs-enable false --velocity-enable false
        """
    )
    
    # Range filter
    parser.add_argument('--range-enable', type=lambda x: x.lower() == 'true',
                        help='Enable range filter (true/false)')
    parser.add_argument('--range-min', type=float, help='Minimum range (m)')
    parser.add_argument('--range-max', type=float, help='Maximum range (m)')
    
    # RCS filter
    parser.add_argument('--rcs-enable', type=lambda x: x.lower() == 'true',
                        help='Enable RCS filter (true/false)')
    parser.add_argument('--rcs-min', type=float, help='Minimum RCS (dBsm)')
    parser.add_argument('--rcs-max', type=float, help='Maximum RCS (dBsm)')
    
    # Velocity filter
    parser.add_argument('--velocity-enable', type=lambda x: x.lower() == 'true',
                        help='Enable velocity filter (true/false)')
    parser.add_argument('--velocity-threshold', type=float, 
                        help='Velocity absolute threshold (m/s)')
    parser.add_argument('--velocity-more', type=lambda x: x.lower() == 'true',
                        help='true: keep |v| > threshold, false: keep |v| < threshold')
    
    # Height filter
    parser.add_argument('--height-enable', type=lambda x: x.lower() == 'true',
                        help='Enable height filter (true/false)')
    parser.add_argument('--height-min', type=float, help='Minimum height (m)')
    parser.add_argument('--height-max', type=float, help='Maximum height (m)')
    
    # X filter
    parser.add_argument('--x-enable', type=lambda x: x.lower() == 'true',
                        help='Enable X filter (true/false)')
    parser.add_argument('--x-min', type=float, help='Minimum X (m)')
    parser.add_argument('--x-max', type=float, help='Maximum X (m)')
    
    # Y filter
    parser.add_argument('--y-enable', type=lambda x: x.lower() == 'true',
                        help='Enable Y filter (true/false)')
    parser.add_argument('--y-min', type=float, help='Minimum Y (m)')
    parser.add_argument('--y-max', type=float, help='Maximum Y (m)')
    
    # Azimuth filter
    parser.add_argument('--azimuth-enable', type=lambda x: x.lower() == 'true',
                        help='Enable azimuth filter (true/false)')
    parser.add_argument('--azimuth-min', type=float, help='Minimum azimuth (deg)')
    parser.add_argument('--azimuth-max', type=float, help='Maximum azimuth (deg)')
    
    # Elevation filter
    parser.add_argument('--elevation-enable', type=lambda x: x.lower() == 'true',
                        help='Enable elevation filter (true/false)')
    parser.add_argument('--elevation-min', type=float, help='Minimum elevation (deg)')
    parser.add_argument('--elevation-max', type=float, help='Maximum elevation (deg)')
    
    # SNR filter
    parser.add_argument('--snr-enable', type=lambda x: x.lower() == 'true',
                        help='Enable SNR filter (true/false)')
    parser.add_argument('--snr-min', type=float, help='Minimum SNR (dB)')
    parser.add_argument('--snr-max', type=float, help='Maximum SNR (dB)')
    
    # Power filter
    parser.add_argument('--power-enable', type=lambda x: x.lower() == 'true',
                        help='Enable power filter (true/false)')
    parser.add_argument('--power-min', type=float, help='Minimum power (dB)')
    parser.add_argument('--power-max', type=float, help='Maximum power (dB)')
    
    # Noise filter
    parser.add_argument('--noise-enable', type=lambda x: x.lower() == 'true',
                        help='Enable noise filter (true/false)')
    parser.add_argument('--noise-min', type=float, help='Minimum noise (dB)')
    parser.add_argument('--noise-max', type=float, help='Maximum noise (dB)')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init(args=sys.argv)
    node = Node('filter_service_client')
    
    # Create service client
    cli = node.create_client(SetRadarPointsFilter, '/filter_node/set_filter')
    
    # Wait for service
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service /filter_node/set_filter not available!')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    
    # Create request with current values or defaults
    request = SetRadarPointsFilter.Request()
    
    # Set default values (current filter state will be preserved if not specified)
    # These defaults match the filter_node defaults
    request.enable_range_filter = args.range_enable if args.range_enable is not None else False
    request.range_min = args.range_min if args.range_min is not None else 0.0
    request.range_max = args.range_max if args.range_max is not None else 100.0
    
    request.enable_rcs_filter = args.rcs_enable if args.rcs_enable is not None else False
    request.rcs_min = args.rcs_min if args.rcs_min is not None else -50.0
    request.rcs_max = args.rcs_max if args.rcs_max is not None else 50.0
    
    request.enable_velocity_filter = args.velocity_enable if args.velocity_enable is not None else False
    request.velocity_abs_threshold = args.velocity_threshold if args.velocity_threshold is not None else 0.5
    request.velocity_filter_more = args.velocity_more if args.velocity_more is not None else True
    
    request.enable_azimuth_filter = args.azimuth_enable if args.azimuth_enable is not None else False
    request.azimuth_min = args.azimuth_min if args.azimuth_min is not None else -180.0
    request.azimuth_max = args.azimuth_max if args.azimuth_max is not None else 180.0
    
    request.enable_elevation_filter = args.elevation_enable if args.elevation_enable is not None else False
    request.elevation_min = args.elevation_min if args.elevation_min is not None else -90.0
    request.elevation_max = args.elevation_max if args.elevation_max is not None else 90.0
    
    request.enable_height_filter = args.height_enable if args.height_enable is not None else False
    request.height_min = args.height_min if args.height_min is not None else -10.0
    request.height_max = args.height_max if args.height_max is not None else 10.0
    
    request.enable_x_filter = args.x_enable if args.x_enable is not None else False
    request.x_min = args.x_min if args.x_min is not None else -100.0
    request.x_max = args.x_max if args.x_max is not None else 100.0
    
    request.enable_y_filter = args.y_enable if args.y_enable is not None else False
    request.y_min = args.y_min if args.y_min is not None else -100.0
    request.y_max = args.y_max if args.y_max is not None else 100.0
    
    request.enable_snr_filter = args.snr_enable if args.snr_enable is not None else False
    request.snr_min = args.snr_min if args.snr_min is not None else 0.0
    request.snr_max = args.snr_max if args.snr_max is not None else 100.0
    
    request.enable_power_filter = args.power_enable if args.power_enable is not None else False
    request.power_min = args.power_min if args.power_min is not None else -100.0
    request.power_max = args.power_max if args.power_max is not None else 100.0
    
    request.enable_noise_filter = args.noise_enable if args.noise_enable is not None else False
    request.noise_min = args.noise_min if args.noise_min is not None else -100.0
    request.noise_max = args.noise_max if args.noise_max is not None else 100.0
    
    # Call service
    node.get_logger().info('Calling filter service...')
    future = cli.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() is not None:
        response = future.result()
        if response.success:
            node.get_logger().info(f'✓ {response.message}')
            ret_code = 0
        else:
            node.get_logger().error(f'✗ {response.message}')
            ret_code = 1
    else:
        node.get_logger().error('Service call failed')
        ret_code = 1
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    
    return ret_code


if __name__ == '__main__':
    sys.exit(main())

