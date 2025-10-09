#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import math


class KalmanFilter:
    """
    Kalman Filter for tracking object position and velocity in 3D space.
    State vector: [x, y, z, vx, vy, vz]
    """
    
    def __init__(self, dt=0.1, process_noise=1.0, measurement_noise=0.5):
        """
        Initialize Kalman Filter.
        
        Args:
            dt: Time step between measurements
            process_noise: Process noise covariance
            measurement_noise: Measurement noise covariance
        """
        self.dt = dt
        
        # State vector [x, y, z, vx, vy, vz]
        self.x = np.zeros((6, 1))
        
        # State covariance matrix
        self.P = np.eye(6) * 10.0
        
        # State transition matrix
        self.F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])
        
        # Measurement matrix (we measure position only)
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # Process noise covariance
        self.Q = np.eye(6) * process_noise
        
        # Measurement noise covariance
        self.R = np.eye(3) * measurement_noise
        
        self.initialized = False
        self.last_update_time = None
    
    def initialize(self, measurement):
        """Initialize filter with first measurement."""
        self.x[0:3] = measurement.reshape(3, 1)
        self.x[3:6] = 0  # Initial velocity is zero
        self.initialized = True
    
    def predict(self):
        """Predict step of Kalman filter."""
        # Predict state
        self.x = self.F @ self.x
        
        # Predict covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x[0:3].flatten()
    
    def update(self, measurement):
        """Update step of Kalman filter."""
        if not self.initialized:
            self.initialize(measurement)
            return self.x[0:3].flatten()
        
        # Measurement residual
        z = measurement.reshape(3, 1)
        y = z - self.H @ self.x
        
        # Residual covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P
        
        return self.x[0:3].flatten()


class TrackedObject:
    """Represents a tracked object with Kalman filter."""
    
    def __init__(self, object_id, initial_pos, dt=0.1, process_noise=1.0, measurement_noise=0.5):
        self.object_id = object_id
        self.kf = KalmanFilter(dt, process_noise, measurement_noise)
        self.kf.initialize(initial_pos)
        self.last_seen = None
        self.missed_updates = 0
        self.position = initial_pos
        self.predicted_position = initial_pos


class KalmanFilterNode(Node):
    """
    ROS2 node that applies Kalman filtering to radar dynamic markers
    to predict object positions.
    """
    
    def __init__(self):
        super().__init__('kalman_filter_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_topic', '/radar_dynamic_markers'),
                ('output_topic', '/Kalman_predict'),
                ('dt', 0.1),  # Time step between measurements
                ('process_noise', 1.0),  # Process noise covariance
                ('measurement_noise', 0.5),  # Measurement noise covariance
                ('association_threshold', 2.0),  # Max distance for data association (meters)
                ('max_missed_updates', 5),  # Max missed updates before removing object
                ('marker_lifetime', 1.0),  # Marker lifetime in seconds
                ('prediction_steps', 5),  # Number of prediction steps to visualize
            ]
        )
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.dt = self.get_parameter('dt').value
        self.process_noise = self.get_parameter('process_noise').value
        self.measurement_noise = self.get_parameter('measurement_noise').value
        self.association_threshold = self.get_parameter('association_threshold').value
        self.max_missed_updates = self.get_parameter('max_missed_updates').value
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        self.prediction_steps = self.get_parameter('prediction_steps').value
        
        # Dictionary to store tracked objects
        self.tracked_objects = {}
        self.next_object_id = 0
        
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
        
        # Create publisher for predicted markers
        self.pub = self.create_publisher(
            MarkerArray,
            self.output_topic,
            10
        )
        
        self.get_logger().info('=' * 80)
        self.get_logger().info('Kalman Filter Node initialized')
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'  Input:  {self.input_topic}')
        self.get_logger().info(f'  Output: {self.output_topic}')
        self.get_logger().info(f'  dt: {self.dt:.3f}s')
        self.get_logger().info(f'  Process noise: {self.process_noise}')
        self.get_logger().info(f'  Measurement noise: {self.measurement_noise}')
        self.get_logger().info(f'  Association threshold: {self.association_threshold}m')
        self.get_logger().info(f'  Max missed updates: {self.max_missed_updates}')
        self.get_logger().info(f'  Prediction steps: {self.prediction_steps}')
        self.get_logger().info('=' * 80)
    
    def euclidean_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two 3D positions."""
        return math.sqrt(
            (pos1[0] - pos2[0])**2 +
            (pos1[1] - pos2[1])**2 +
            (pos1[2] - pos2[2])**2
        )
    
    def associate_measurements(self, measurements):
        """
        Associate measurements with tracked objects using nearest neighbor.
        
        Returns:
            dict: Mapping of object_id to measurement index
            list: Indices of unassociated measurements
        """
        associations = {}
        unassociated = list(range(len(measurements)))
        
        # Predict all tracked objects
        predictions = {}
        for obj_id, tracked_obj in self.tracked_objects.items():
            predictions[obj_id] = tracked_obj.kf.predict()
        
        # Associate measurements to predictions
        for obj_id, predicted_pos in predictions.items():
            best_match = None
            best_distance = self.association_threshold
            
            for i in unassociated[:]:
                distance = self.euclidean_distance(predicted_pos, measurements[i])
                if distance < best_distance:
                    best_distance = distance
                    best_match = i
            
            if best_match is not None:
                associations[obj_id] = best_match
                unassociated.remove(best_match)
        
        return associations, unassociated
    
    def marker_callback(self, msg):
        """Callback for MarkerArray messages."""
        if len(msg.markers) == 0:
            return
        
        current_time = self.get_clock().now()
        
        # Extract measurements (positions) from markers
        measurements = []
        for marker in msg.markers:
            pos = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])
            measurements.append(pos)
        
        # Data association
        associations, unassociated = self.associate_measurements(measurements)
        
        # Update associated objects
        updated_objects = set()
        for obj_id, meas_idx in associations.items():
            tracked_obj = self.tracked_objects[obj_id]
            updated_pos = tracked_obj.kf.update(measurements[meas_idx])
            tracked_obj.position = updated_pos
            tracked_obj.last_seen = current_time
            tracked_obj.missed_updates = 0
            updated_objects.add(obj_id)
        
        # Create new objects for unassociated measurements
        for meas_idx in unassociated:
            new_obj = TrackedObject(
                self.next_object_id,
                measurements[meas_idx],
                self.dt,
                self.process_noise,
                self.measurement_noise
            )
            new_obj.last_seen = current_time
            self.tracked_objects[self.next_object_id] = new_obj
            updated_objects.add(self.next_object_id)
            self.next_object_id += 1
        
        # Increment missed updates for objects not updated
        objects_to_remove = []
        for obj_id in self.tracked_objects:
            if obj_id not in updated_objects:
                self.tracked_objects[obj_id].missed_updates += 1
                if self.tracked_objects[obj_id].missed_updates > self.max_missed_updates:
                    objects_to_remove.append(obj_id)
        
        # Remove objects that have been missed too many times
        for obj_id in objects_to_remove:
            del self.tracked_objects[obj_id]
        
        # Publish predicted positions
        self.publish_predictions(msg.markers[0].header)
        
        self.get_logger().debug(
            f'Tracking {len(self.tracked_objects)} objects | '
            f'Updated: {len(updated_objects)} | '
            f'New: {len(unassociated)} | '
            f'Removed: {len(objects_to_remove)}'
        )
    
    def publish_predictions(self, header):
        """Publish predicted object positions as markers."""
        marker_array = MarkerArray()
        marker_id = 0
        
        for obj_id, tracked_obj in self.tracked_objects.items():
            # Current filtered position
            filtered_marker = self.create_marker(
                header,
                marker_id,
                tracked_obj.position,
                'filtered',
                [0.0, 1.0, 0.0, 0.8]  # Green for filtered position
            )
            marker_array.markers.append(filtered_marker)
            marker_id += 1
            
            # Predict future positions
            kf_copy = KalmanFilter(self.dt, self.process_noise, self.measurement_noise)
            kf_copy.x = tracked_obj.kf.x.copy()
            kf_copy.P = tracked_obj.kf.P.copy()
            kf_copy.F = tracked_obj.kf.F.copy()
            kf_copy.initialized = True
            
            for step in range(1, self.prediction_steps + 1):
                predicted_pos = kf_copy.predict()
                
                # Fade alpha with prediction steps
                alpha = 0.8 - (step / self.prediction_steps) * 0.5
                
                pred_marker = self.create_marker(
                    header,
                    marker_id,
                    predicted_pos,
                    f'prediction_{step}',
                    [1.0, 0.5, 0.0, alpha]  # Orange for predictions, fading
                )
                marker_array.markers.append(pred_marker)
                marker_id += 1
        
        # Publish
        if len(marker_array.markers) > 0:
            self.pub.publish(marker_array)
    
    def create_marker(self, header, marker_id, position, namespace, color):
        """Create a visualization marker."""
        marker = Marker()
        marker.header = header
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        # Color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

