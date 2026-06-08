import os
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from prisma_rover_quantum_controller.lut_loader import LUT2D
from prisma_rover_quantum_controller.utils import normalize_angle, build_twist
from tf2_ros import Buffer, TransformListener, TransformException

class QuantumControllerNode(Node):
    def __init__(self):
        super().__init__('quantum_controller')

        # --------------------- ROS 2 Parameters ---------------------
        self.declare_parameter('map_frame', 'prisma_rover/map')
        self.declare_parameter('base_frame', 'prisma_rover/base_footprint')
        self.declare_parameter('lut_nx', 100)
        self.declare_parameter('lut_ny', 100)
        self.declare_parameter('lut_method', 'linear')
        self.declare_parameter('config_dir', '')

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.lut_nx = self.get_parameter('lut_nx').get_parameter_value().integer_value
        self.lut_ny = self.get_parameter('lut_ny').get_parameter_value().integer_value
        self.lut_method = self.get_parameter('lut_method').get_parameter_value().string_value

        config_dir_param = self.get_parameter('config_dir').get_parameter_value().string_value
        if config_dir_param:
            self.config_dir = config_dir_param
        else:
            self.config_dir = os.path.join(get_package_share_directory('prisma_rover_quantum_controller'), 'config')

        # --------------------- TF2 setup ---------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --------------------- Pub/Sub (Relative topics) ---------------------
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.fuzzy_input_pub = self.create_publisher(Vector3, 'fuzzy_input', 10)
        self.robot_pose_pub = self.create_publisher(Vector3, 'position', 10)
        
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.obstacle_sub = self.create_subscription(Float32, 'obstacle', self.obstacle_callback, 10)
        self.min_dist_sub = self.create_subscription(Float32, 'min_distance', self.min_distance_callback, 10)

        # --------------------- Internal state ---------------------
        self.goal_received = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.obstacle_angle = 3.14
        self.min_distance = 10.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        # --------------------- Load LUTs via shared loader ---------------------
        self.linear_LUT = LUT2D(
            os.path.join(self.config_dir, 'LUT_lin.csv'),
            ['dist_error', 'obstacle', 'linear_velocity'],
            logger=self.get_logger(),
            nx=self.lut_nx,
            ny=self.lut_ny,
            method=self.lut_method
        )
        self.angular_LUT = LUT2D(
            os.path.join(self.config_dir, 'LUT_ang.csv'),
            ['orient_error', 'obstacle', 'angular_velocity'],
            logger=self.get_logger(),
            nx=self.lut_nx,
            ny=self.lut_ny,
            method=self.lut_method
        )

        self.get_logger().info("✅ Quantum Controller Node initialized with namespaced parameters")

    def update_pose_from_tf(self) -> bool:
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1)
            )
        except TransformException as ex:
            if not hasattr(self, "_last_tf_warn") or (self.get_clock().now() - self._last_tf_warn).nanoseconds > 5e8:
                self.get_logger().warn(f"TF not available ({self.map_frame} <- {self.base_frame}): {ex}")
                self._last_tf_warn = self.get_clock().now()
            return False

        # Extract translation
        self.robot_x = trans.transform.translation.x
        self.robot_y = trans.transform.translation.y

        # Extract yaw from quaternion
        q = trans.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Publish pose
        msg = Vector3(x=self.robot_x, y=self.robot_y, z=0.0)
        self.robot_pose_pub.publish(msg)
        return True

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_received = True
        self.get_logger().info(f"🎯 New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def obstacle_callback(self, msg: Float32):
        self.obstacle_angle = msg.data

    def min_distance_callback(self, msg: Float32):
        self.min_distance = msg.data

    def control_loop(self):
        if not self.goal_received:
            return

        if not self.update_pose_from_tf():
            return

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist_error = math.sqrt(dx ** 2 + dy ** 2)
        target_angle = math.atan2(dy, dx)
        orient_error = normalize_angle(target_angle - self.robot_yaw)

        # Interpolate LUT commands (scaling applied for Differential Drive constraints)
        v = self.linear_LUT.query(dist_error, self.obstacle_angle) * 1.5 / 5.0
        w = self.angular_LUT.query(orient_error, self.obstacle_angle) * 1.8 / 5.0

        if dist_error < 0.05:
            v, w = 0.0, 0.0
            self.goal_received = False
            self.get_logger().info("🎯 Goal reached!")

        # Publish velocities
        self.cmd_pub.publish(build_twist(v, w))

        # Publish debug fuzzy input
        self.fuzzy_input_pub.publish(Vector3(x=dist_error, y=target_angle, z=self.obstacle_angle))

        # Logging
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if not hasattr(self, "_last_log_time"):
            self._last_log_time = 0.0
        if now - self._last_log_time > 1.0:
            self.get_logger().info(
                f"Pose: ({self.robot_x:.2f}, {self.robot_y:.2f}, {self.robot_yaw:.2f}) | "
                f"DistErr: {dist_error:.2f}, ObsAngle: {self.obstacle_angle:.2f} -> CmdVel: v={v:.2f}, w={w:.2f}"
            )
            self._last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = QuantumControllerNode()
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
