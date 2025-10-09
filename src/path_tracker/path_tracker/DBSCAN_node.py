#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int32, Header
import numpy as np
from sklearn.cluster import DBSCAN
import struct


class DBSCANNode(Node):
    """
    ROS2 node that performs DBSCAN clustering on radar dynamic markers.
    Publishes the number of clusters and cluster centers.
    """
    
    def __init__(self):
        super().__init__('dbscan_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Topics configuration
                ('input_topic', '/radar_dynamic_markers'),
                ('clusters_num_topic', '/clusters_num'),
                ('clusters_centers_topic', '/clusters_centers'),
                ('clusters_markers_topic', '/clusters_markers'),
                
                # DBSCAN parameters
                ('eps', 1.0),  # Maximum distance between two samples
                ('min_samples', 2),  # Minimum number of samples in a cluster
                
                # Frame configuration
                ('frame_id', 'base_link'),
                
                # Visualization parameters
                ('publish_markers', True),  # Publish visualization markers
                ('marker_lifetime', 1.0),  # Marker lifetime in seconds
            ]
        )
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.clusters_num_topic = self.get_parameter('clusters_num_topic').value
        self.clusters_centers_topic = self.get_parameter('clusters_centers_topic').value
        self.clusters_markers_topic = self.get_parameter('clusters_markers_topic').value
        self.eps = self.get_parameter('eps').value
        self.min_samples = self.get_parameter('min_samples').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_markers = self.get_parameter('publish_markers').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        
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
        
        # Create publishers
        self.num_pub = self.create_publisher(
            Int32,
            self.clusters_num_topic,
            10
        )
        
        self.centers_pub = self.create_publisher(
            PointCloud2,
            self.clusters_centers_topic,
            10
        )
        
        if self.publish_markers:
            self.markers_pub = self.create_publisher(
                MarkerArray,
                self.clusters_markers_topic,
                10
            )
        
        # Statistics
        self.total_processed = 0
        self.total_clusters = 0
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('DBSCAN Clustering Node initialized')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'  Input:  {self.input_topic}')
        self.get_logger().info(f'  Output: {self.clusters_num_topic} (clusters count)')
        self.get_logger().info(f'  Output: {self.clusters_centers_topic} (cluster centers)')
        if self.publish_markers:
            self.get_logger().info(f'  Output: {self.clusters_markers_topic} (visualization)')
        self.get_logger().info('-' * 80)
        self.get_logger().info(f'  DBSCAN eps: {self.eps}')
        self.get_logger().info(f'  DBSCAN min_samples: {self.min_samples}')
        self.get_logger().info(f'  Frame ID: {self.frame_id}')
        self.get_logger().info('=' * 80)
    
    def marker_callback(self, msg):
        """Callback for MarkerArray messages."""
        if len(msg.markers) == 0:
            # No markers, publish zero clusters
            self.publish_results(msg.markers[0].header if len(msg.markers) > 0 else None, [], [])
            return
        
        # Extract positions from markers
        points = []
        for marker in msg.markers:
            points.append([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])
        
        points = np.array(points)
        
        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = clustering.labels_
        
        # Number of clusters (excluding noise points labeled as -1)
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise = list(labels).count(-1)
        
        # Calculate cluster centers
        cluster_centers = []
        cluster_labels = []
        for cluster_id in range(n_clusters):
            cluster_mask = labels == cluster_id
            cluster_points = points[cluster_mask]
            center = np.mean(cluster_points, axis=0)
            cluster_centers.append(center)
            cluster_labels.append(cluster_id)
        
        # Update statistics
        self.total_processed += len(points)
        self.total_clusters += n_clusters
        
        # Publish results
        self.publish_results(msg.markers[0].header, cluster_centers, cluster_labels)
        
        # Optionally publish visualization markers
        if self.publish_markers and n_clusters > 0:
            self.publish_cluster_markers(msg.markers[0].header, points, labels, cluster_centers)
        
        self.get_logger().debug(
            f'Processed {len(points)} points | '
            f'Found {n_clusters} clusters | '
            f'Noise points: {n_noise}'
        )
    
    def publish_results(self, header, cluster_centers, cluster_labels):
        """Publish clustering results."""
        # Create header if not provided
        if header is None:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.frame_id
        
        # Publish number of clusters
        num_msg = Int32()
        num_msg.data = len(cluster_centers)
        self.num_pub.publish(num_msg)
        
        # Publish cluster centers as PointCloud2
        if len(cluster_centers) > 0:
            centers_pc2 = self.create_pointcloud2(header, np.array(cluster_centers))
            self.centers_pub.publish(centers_pc2)
        else:
            # Publish empty point cloud
            centers_pc2 = self.create_pointcloud2(header, np.array([]))
            self.centers_pub.publish(centers_pc2)
    
    def create_pointcloud2(self, header, points):
        """
        Create a PointCloud2 message from numpy array.
        
        Args:
            header: ROS message header
            points: Nx3 numpy array of points (x, y, z)
        
        Returns:
            PointCloud2 message
        """
        msg = PointCloud2()
        msg.header = header
        
        if len(points) == 0:
            msg.height = 1
            msg.width = 0
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = 0
            msg.is_dense = True
            msg.data = b''
            return msg
        
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack points into binary data
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', float(point[0]), float(point[1]), float(point[2])))
        
        msg.data = b''.join(buffer)
        
        return msg
    
    def publish_cluster_markers(self, header, points, labels, cluster_centers):
        """Publish visualization markers for clusters."""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Generate colors for each cluster
        n_clusters = len(cluster_centers)
        colors = self.generate_colors(n_clusters)
        
        # Publish markers for each point colored by cluster
        for i, point in enumerate(points):
            if labels[i] == -1:
                # Noise point - skip or mark in gray
                continue
            
            marker = Marker()
            marker.header = header
            marker.ns = "cluster_points"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = float(point[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            color = colors[labels[i]]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.8
            
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        # Publish markers for cluster centers
        for i, center in enumerate(cluster_centers):
            marker = Marker()
            marker.header = header
            marker.ns = "cluster_centers"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            color = colors[i]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        if len(marker_array.markers) > 0:
            self.markers_pub.publish(marker_array)
    
    def generate_colors(self, n_colors):
        """Generate distinct colors for clusters."""
        colors = []
        for i in range(n_colors):
            hue = i / max(n_colors, 1)
            # HSV to RGB conversion (simplified)
            r, g, b = self.hsv_to_rgb(hue, 0.8, 0.9)
            colors.append([r, g, b])
        return colors
    
    def hsv_to_rgb(self, h, s, v):
        """Convert HSV color to RGB."""
        import colorsys
        return colorsys.hsv_to_rgb(h, s, v)


def main(args=None):
    rclpy.init(args=args)
    node = DBSCANNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

