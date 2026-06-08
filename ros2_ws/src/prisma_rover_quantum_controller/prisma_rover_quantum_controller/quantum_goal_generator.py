import os
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from ament_index_python.packages import get_package_share_directory

from prisma_rover_quantum_controller.lut_loader import LUT2D
from prisma_rover_quantum_controller.utils import normalize_angle

class QuantumGoalGenerator(Node):
    def __init__(self):
        super().__init__('quantum_goal_generator')

        # --------------------- Parameters ---------------------
        self.declare_parameter('config_dir', '')
        config_dir_param = self.get_parameter('config_dir').get_parameter_value().string_value
        if config_dir_param:
            self.config_dir = config_dir_param
        else:
            self.config_dir = os.path.join(get_package_share_directory('prisma_rover_quantum_controller'), 'config')

        # Subscribers (Relative topics)
        self.odom_sub = self.create_subscription(Odometry, 'odom/wheels', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.obstacle_sub = self.create_subscription(Float32, 'obstacle', self.obstacle_callback, 10)

        # Publisher for local planner (Relative topic)
        self.plan_pub = self.create_publisher(Path, 'plan', 10)

        # Internal state
        self.goal_received = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.obstacle_angle = 0.0

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Load position offset LUTs
        self.LUT_x = LUT2D(
            os.path.join(self.config_dir, 'LUT_POS_X.csv'),
            ['dist_error', 'obstacle', 'pos_x'],
            logger=self.get_logger()
        )
        self.LUT_y = LUT2D(
            os.path.join(self.config_dir, 'LUT_POS_Y.csv'),
            ['orient_error', 'obstacle', 'pos_y'],
            logger=self.get_logger()
        )

        self.get_logger().info("✅ Quantum Goal Generator initialized. Publishing local subgoals to topic: plan")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_received = True
        self.get_logger().info(f"🎯 Global goal updated: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def obstacle_callback(self, msg: Float32):
        self.obstacle_angle = msg.data

    def control_loop(self):
        if not self.goal_received:
            return

        # Distance and bearing to global goal
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        goal_dist = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        orient_error = normalize_angle(target_angle - self.robot_yaw)

        # Look up local offsets relative to robot coordinate frame
        offset_x = self.LUT_x.query(goal_dist, self.obstacle_angle)
        offset_y = self.LUT_y.query(orient_error, self.obstacle_angle)

        # Project intermediate goal into global map frame
        local_goal_x = self.robot_x + math.cos(self.robot_yaw) * offset_x - math.sin(self.robot_yaw) * offset_y
        local_goal_y = self.robot_y + math.sin(self.robot_yaw) * offset_x + math.cos(self.robot_yaw) * offset_y

        # Build Path message
        local_pose = PoseStamped()
        local_pose.header.frame_id = 'prisma_rover/map'
        local_pose.header.stamp = self.get_clock().now().to_msg()
        local_pose.pose.position.x = local_goal_x
        local_pose.pose.position.y = local_goal_y
        local_pose.pose.orientation.w = 1.0

        path_msg = Path()
        path_msg.header.frame_id = 'prisma_rover/map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = [local_pose]

        self.plan_pub.publish(path_msg)

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if not hasattr(self, "_last_log_time"):
            self._last_log_time = 0.0
        if now - self._last_log_time > 1.0:
            self.get_logger().info(
                f"GlobalGoal: ({self.goal_x:.2f}, {self.goal_y:.2f}) | "
                f"LocalGoal: ({local_goal_x:.2f}, {local_goal_y:.2f}) -> plan"
            )
            self._last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = QuantumGoalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
