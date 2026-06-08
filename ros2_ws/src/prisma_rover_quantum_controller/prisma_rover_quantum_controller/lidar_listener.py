import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2 as pc2
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ClosestObstacleFromLidar(Node):
    def __init__(self):
        super().__init__('closest_obstacle_from_lidar')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        self.declare_parameter('lidar_topic', '')  # 3D PointCloud2 topic (defaults dynamically)
        self.declare_parameter('scan_topic', 'scan')  # 2D LaserScan topic
        self.declare_parameter('elevation_band_deg', 5.0)  # in degrees (only for 3D)
        self.declare_parameter('lateral_gate_width', 0.35)
        self.declare_parameter('max_distance', 1.0)
        self.declare_parameter('reliability', 'best_effort')
        
        # Dynamic default for 3D LiDAR topic based on use_sim_time
        lidar_topic_param = self.get_parameter('lidar_topic').get_parameter_value().string_value
        if not lidar_topic_param:
            self.lidar_topic = 'scan_3d' if use_sim_time else 'livox/lidar'
        else:
            self.lidar_topic = lidar_topic_param

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        
        elev_band_deg = self.get_parameter('elevation_band_deg').get_parameter_value().double_value
        self.elev_band_rad = math.radians(elev_band_deg)
        self.lateral_gate_width = self.get_parameter('lateral_gate_width').get_parameter_value().double_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value

        reliability_str = self.get_parameter('reliability').get_parameter_value().string_value.lower()
        if reliability_str == 'reliable':
            reliability_policy = ReliabilityPolicy.RELIABLE
        else:
            reliability_policy = ReliabilityPolicy.BEST_EFFORT

        # QoS profile matching simulator/hardware
        qos_profile = QoSProfile(
            reliability=reliability_policy,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --------------------- Subscriptions ---------------------
        # 3D Lidar PointCloud2 Subscription
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.cloud_callback,
            qos_profile
        )

        # 2D Lidar LaserScan Subscription
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile
        )

        # --------------------- Publishers ---------------------
        self.publisher = self.create_publisher(Float32, 'obstacle', 10)
        self.min_dist_publisher = self.create_publisher(Float32, 'min_distance', 10)

        self.get_logger().info(
            f"✅ Lidar Listener initialized. "
            f"Subscribed to 3D Topic: '{self.lidar_topic}' & 2D Topic: '{self.scan_topic}'"
        )

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Filter out invalid / out-of-range measurements
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        if not np.any(valid):
            self._publish_neutral()
            return

        ranges = ranges[valid]
        angles = angles[valid]

        # Convert to 3D Cartesian coordinates relative to sensor frame (Z is 0)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)

        self.process_points(x, y, z, ranges)

    def cloud_callback(self, msg: PointCloud2):
        # Check if point cloud has expected layout (x, y, z are float32 at offsets 0, 4, 8)
        try:
            # Create numpy array from byte buffer
            data_arr = np.frombuffer(msg.data, dtype=np.uint8)
            data_arr = data_arr.reshape(-1, msg.point_step)
            # View fields x, y, z as float32
            xyz = data_arr[:, :12].view(dtype=np.float32)
            x = xyz[:, 0]
            y = xyz[:, 1]
            z = xyz[:, 2]
            
            # Filter NaNs
            non_nan = ~(np.isnan(x) | np.isnan(y) | np.isnan(z))
            x = x[non_nan]
            y = y[non_nan]
            z = z[non_nan]
        except Exception as e:
            self.get_logger().error(f"Fast point cloud parsing failed, falling back to pc2: {e}")
            # Fallback to slow pc2 method if layout is different
            try:
                points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                if not points_list:
                    self._publish_neutral()
                    return
                points = np.array(points_list)
                if points.ndim == 2 and points.shape[1] >= 3:
                    x = points[:, 0]
                    y = points[:, 1]
                    z = points[:, 2]
                else:
                    x = points['x']
                    y = points['y']
                    z = points['z']
                # Filter NaNs (just in case)
                non_nan = ~(np.isnan(x) | np.isnan(y) | np.isnan(z))
                x = x[non_nan]
                y = y[non_nan]
                z = z[non_nan]
            except Exception as ex:
                self.get_logger().error(f"Fallback reading point cloud failed: {ex}")
                self._publish_neutral()
                return

        # Exclude exact origin points (0, 0, 0)
        non_zero = (x != 0.0) | (y != 0.0) | (z != 0.0)
        if not np.any(non_zero):
            self._publish_neutral()
            return

        x, y, z = x[non_zero], y[non_zero], z[non_zero]

        # Compute projection and 3D distances
        r_xy = np.hypot(x, y)
        distance = np.hypot(r_xy, z)

        # Elevation angle thresholding
        elev = np.arctan2(z, r_xy)
        elev_mask = np.abs(elev) <= self.elev_band_rad

        # Filter out points outside elevation band
        x, y, z, distance = x[elev_mask], y[elev_mask], z[elev_mask], distance[elev_mask]

        self.process_points(x, y, z, distance)

    def process_points(self, x, y, z, distance):
        if len(x) == 0:
            self._publish_neutral()
            return

        # Global minimum distance (excluding origin noise)
        global_min_distance = np.min(distance)

        # Gating calculations for obstacle identification in front of the robot
        theta = np.arctan2(y, x)

        # Lateral gate widens slightly as distance decreases to mimic cone projection
        lateral_gate = self.lateral_gate_width + 0.20 * ((self.max_distance - distance) / self.max_distance)
        gating_mask = (distance <= self.max_distance) & (np.abs(y) <= lateral_gate) & (theta >= -1.6) & (theta <= 1.6)

        if np.any(gating_mask):
            matched_distances = distance[gating_mask]
            matched_thetas = theta[gating_mask]
            closest_idx = np.argmin(matched_distances)
            min_bearing = matched_thetas[closest_idx]
        else:
            min_bearing = None

        # Publish angle of closest obstacle (use fallback neutral value of 3.0 if no obstacle)
        angle_to_publish = 3.0 if min_bearing is None else float(min_bearing)
        self.publisher.publish(Float32(data=angle_to_publish))

        # Publish global min distance
        self.min_dist_publisher.publish(Float32(data=float(global_min_distance)))

    def _publish_neutral(self):
        self.publisher.publish(Float32(data=3.0))
        self.min_dist_publisher.publish(Float32(data=10.0))

def main(args=None):
    rclpy.init(args=args)
    node = ClosestObstacleFromLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
