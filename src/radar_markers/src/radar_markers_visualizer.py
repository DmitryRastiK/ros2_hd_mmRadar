#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class ObjectClass:
    """Represents an object class with RCS range and marker properties."""
    def __init__(self, name, rcs_min, rcs_max, marker_type, 
                 size_x, size_y, size_z, color_r, color_g, color_b, color_a):
        self.name = name
        self.rcs_min = rcs_min
        self.rcs_max = rcs_max
        self.marker_type = marker_type
        self.size_x = size_x
        self.size_y = size_y
        self.size_z = size_z
        self.color_r = color_r
        self.color_g = color_g
        self.color_b = color_b
        self.color_a = color_a
    
    def matches(self, rcs):
        """Check if RCS value belongs to this class."""
        return self.rcs_min <= rcs <= self.rcs_max


class RadarMarkersVisualizer(Node):
    """Visualizes radar point cloud as markers with object class classification."""

    def __init__(self):
        super().__init__('radar_markers_visualizer')
        
        # Declare basic parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_topic', '/hd_radar_0/points/static_f'),
                ('output_topic', '/radar_markers'),
                ('marker_lifetime', 1.0),
                ('num_classes', 2),
            ]
        )
        
        # Get basic parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        num_classes = self.get_parameter('num_classes').value
        
        # Load object classes
        self.object_classes = []
        for i in range(num_classes):
            class_params = self.load_class_params(i)
            if class_params:
                self.object_classes.append(class_params)
        
        # Load default marker settings
        self.load_default_marker_params()
        
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
        
        # Create publisher for markers
        self.pub = self.create_publisher(
            MarkerArray,
            self.output_topic,
            10
        )
        
        # Statistics
        self.total_points = 0
        self.class_counts = {cls.name: 0 for cls in self.object_classes}
        self.class_counts['unclassified'] = 0
        
        self.log_initialization()

    def load_class_params(self, class_idx):
        """Load parameters for a specific object class."""
        prefix = f'class_{class_idx}'
        
        # Declare parameters for this class
        self.declare_parameters(
            namespace='',
            parameters=[
                (f'{prefix}.name', 'unknown'),
                (f'{prefix}.rcs_min', 0.0),
                (f'{prefix}.rcs_max', 1.0),
                (f'{prefix}.marker_type', 'cube'),
                (f'{prefix}.size_x', 0.5),
                (f'{prefix}.size_y', 0.5),
                (f'{prefix}.size_z', 0.5),
                (f'{prefix}.color_r', 1.0),
                (f'{prefix}.color_g', 0.0),
                (f'{prefix}.color_b', 0.0),
                (f'{prefix}.color_a', 0.7),
            ]
        )
        
        try:
            obj_class = ObjectClass(
                name=self.get_parameter(f'{prefix}.name').value,
                rcs_min=self.get_parameter(f'{prefix}.rcs_min').value,
                rcs_max=self.get_parameter(f'{prefix}.rcs_max').value,
                marker_type=self.get_marker_type(
                    self.get_parameter(f'{prefix}.marker_type').value
                ),
                size_x=self.get_parameter(f'{prefix}.size_x').value,
                size_y=self.get_parameter(f'{prefix}.size_y').value,
                size_z=self.get_parameter(f'{prefix}.size_z').value,
                color_r=self.get_parameter(f'{prefix}.color_r').value,
                color_g=self.get_parameter(f'{prefix}.color_g').value,
                color_b=self.get_parameter(f'{prefix}.color_b').value,
                color_a=self.get_parameter(f'{prefix}.color_a').value,
            )
            return obj_class
        except Exception as e:
            self.get_logger().error(f'Failed to load class {class_idx}: {e}')
            return None

    def load_default_marker_params(self):
        """Load default marker parameters for unclassified points."""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('default_marker.marker_type', 'sphere'),
                ('default_marker.size', 0.1),
                ('default_marker.color_r', 0.5),
                ('default_marker.color_g', 0.5),
                ('default_marker.color_b', 0.5),
                ('default_marker.color_a', 0.5),
            ]
        )
        
        self.default_marker_type = self.get_marker_type(
            self.get_parameter('default_marker.marker_type').value
        )
        self.default_size = self.get_parameter('default_marker.size').value
        self.default_color_r = self.get_parameter('default_marker.color_r').value
        self.default_color_g = self.get_parameter('default_marker.color_g').value
        self.default_color_b = self.get_parameter('default_marker.color_b').value
        self.default_color_a = self.get_parameter('default_marker.color_a').value

    def get_marker_type(self, type_str):
        """Convert marker type string to Marker constant."""
        types = {
            'sphere': Marker.SPHERE,
            'cube': Marker.CUBE,
            'cylinder': Marker.CYLINDER,
            'arrow': Marker.ARROW,
        }
        return types.get(type_str.lower(), Marker.SPHERE)

    def classify_point(self, rcs):
        """Classify a point based on its RCS value."""
        for obj_class in self.object_classes:
            if obj_class.matches(rcs):
                return obj_class
        return None

    def log_initialization(self):
        """Log initialization information."""
        self.get_logger().info('=' * 80)
        self.get_logger().info('Radar Markers Visualizer Node initialized')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'  Input:  {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
        self.get_logger().info(f'  Marker lifetime: {self.marker_lifetime:.1f}s')
        self.get_logger().info('-' * 80)
        self.get_logger().info(f'Loaded {len(self.object_classes)} object classes:')
        for obj_class in self.object_classes:
            self.get_logger().info(
                f'  {obj_class.name}: RCS [{obj_class.rcs_min:.1f}, {obj_class.rcs_max:.1f}] dBsm, '
                f'Size: {obj_class.size_x:.2f}x{obj_class.size_y:.2f}x{obj_class.size_z:.2f}m'
            )
        self.get_logger().info('=' * 80)

    def cloud_callback(self, msg):
        """Callback for point cloud messages."""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Parse point cloud
        for point in pc2.read_points(msg, field_names=('x', 'y', 'z', 'rcs'), skip_nans=True):
            x, y, z, rcs = point
            self.total_points += 1
            
            # Classify point by RCS
            obj_class = self.classify_point(rcs)
            
            # Create marker
            marker = Marker()
            marker.header = msg.header
            marker.id = marker_id
            marker.action = Marker.ADD
            
            if obj_class:
                # Use class-specific marker properties
                marker.ns = obj_class.name
                marker.type = obj_class.marker_type
                
                # Size
                marker.scale.x = obj_class.size_x
                marker.scale.y = obj_class.size_y
                marker.scale.z = obj_class.size_z
                
                # Color
                marker.color.r = obj_class.color_r
                marker.color.g = obj_class.color_g
                marker.color.b = obj_class.color_b
                marker.color.a = obj_class.color_a
                
                self.class_counts[obj_class.name] += 1
            else:
                # Use default marker properties
                marker.ns = "unclassified"
                marker.type = self.default_marker_type
                
                # Size
                marker.scale.x = self.default_size
                marker.scale.y = self.default_size
                marker.scale.z = self.default_size
                
                # Color
                marker.color.r = self.default_color_r
                marker.color.g = self.default_color_g
                marker.color.b = self.default_color_b
                marker.color.a = self.default_color_a
                
                self.class_counts['unclassified'] += 1
            
            # Position (point center is on the surface, raise by half height)
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z) + marker.scale.z / 2.0
            marker.pose.orientation.w = 1.0
            
            # Lifetime
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        # Publish markers
        if len(marker_array.markers) > 0:
            self.pub.publish(marker_array)
            
            self.get_logger().debug(
                f'Published {len(marker_array.markers)} markers | '
                f'Total: {self.total_points} points, '
                f'Classes: {self.class_counts}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RadarMarkersVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()