#!/usr/bin/env python3
"""
Goal Navigator - Drobot Navigation VER1

Goal-based autonomous navigation with rule engine.
Receives goal from user (RViz/topic) and navigates safely.
"""
import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from drobot_scan_2d.config import Config
from drobot_scan_2d.rules import RuleEngine


class GoalNavigator(Node):
    """Goal-based autonomous navigator with rule enforcement."""

    def __init__(self):
        super().__init__('goal_navigator')

        # Load rules
        self._init_rule_engine()

        # Initialize components
        self._init_publishers_subscribers()
        self._init_action_clients()
        self._init_state()

        self.get_logger().info('Goal Navigator VER1 started!')
        self.get_logger().info(f'Rules loaded: {len(self.rule_engine.stop_rules)} stop rules')

    def _init_rule_engine(self):
        """Initialize rule engine with rules.yaml (path from ROS parameter)."""
        self.declare_parameter('rules_file', '')
        rules_file = self.get_parameter('rules_file').get_parameter_value().string_value

        if not rules_file:
            self.get_logger().warn('No rules_file parameter set, running without rules')

        self.rule_engine = RuleEngine(rules_file if rules_file else None)

    def _init_publishers_subscribers(self):
        """Set up ROS publishers and subscribers."""
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10
        )

        # Publishers
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

    def _init_action_clients(self):
        """Set up Nav2 action clients."""
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _init_state(self):
        """Initialize state variables."""
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Navigation state
        self.is_navigating = False
        self.current_goal: Optional[Tuple[float, float]] = None
        self.goal_handle = None
        self.nav_start_time: Optional[float] = None

        # Sensor state
        self.min_obstacle_dist = float('inf')
        self.obstacle_direction = 0.0

        # Statistics
        self.goals_attempted = 0
        self.goals_succeeded = 0

        # Timers
        self.create_timer(1.0 / Config.COLLISION_CHECK_RATE, self._check_safety)
        self.create_timer(1.0 / Config.RULE_CHECK_RATE, self._check_rules)
        self.create_timer(5.0, self._log_status)

    # ==================== Callbacks ====================

    def _goal_callback(self, msg: PoseStamped):
        """Handle new goal from RViz or topic."""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        self.get_logger().info(f'Received goal: ({goal_x:.2f}, {goal_y:.2f})')

        # Check if goal is in forbidden zone
        is_forbidden, zone_name = self.rule_engine.check_forbidden_zone(goal_x, goal_y)
        if is_forbidden:
            self.get_logger().warn(f'Goal rejected: inside forbidden zone "{zone_name}"')
            return

        # Cancel current navigation if any
        if self.is_navigating and self.goal_handle:
            self.get_logger().info('Cancelling current goal...')
            self.goal_handle.cancel_goal_async()

        # Send new goal
        self._send_goal(goal_x, goal_y)

    def _odom_callback(self, msg: Odometry):
        """Process odometry data."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _scan_callback(self, msg: LaserScan):
        """Process LiDAR scan for obstacle detection."""
        if not msg.ranges:
            return

        # Find minimum distance in front (±60 degrees)
        num_ranges = len(msg.ranges)
        front_center = num_ranges // 2
        angle_range = int((math.pi / 3) / msg.angle_increment)  # 60 degrees

        front_start = max(0, front_center - angle_range)
        front_end = min(num_ranges - 1, front_center + angle_range)

        min_dist = float('inf')
        min_idx = front_center

        for i in range(front_start, front_end + 1):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max and r < min_dist:
                min_dist = r
                min_idx = i

        self.min_obstacle_dist = min_dist
        self.obstacle_direction = msg.angle_min + min_idx * msg.angle_increment

    # ==================== Navigation ====================

    def _send_goal(self, x: float, y: float):
        """Send navigation goal to Nav2."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return

        self.current_goal = (x, y)
        self.is_navigating = True
        self.nav_start_time = time.time()
        self.goals_attempted += 1

        # Create goal message
        # Note: stamp=Time(0) tells Nav2 to use latest TF (avoids extrapolation errors)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        # goal_msg.pose.header.stamp defaults to Time(0) - use latest TF
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Set orientation towards goal
        angle = math.atan2(y - self.robot_y, x - self.robot_x)
        goal_msg.pose.pose.orientation.z = math.sin(angle / 2)
        goal_msg.pose.pose.orientation.w = math.cos(angle / 2)

        # Publish goal marker
        self._publish_goal_marker(x, y)

        # Send goal
        self.get_logger().info(f'Navigating to ({x:.2f}, {y:.2f})...')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.is_navigating = False
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
            self.goals_succeeded += 1
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal was canceled')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')

        self.is_navigating = False
        self.current_goal = None

    def _publish_goal_marker(self, x: float, y: float):
        """Publish goal visualization marker."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'navigation_goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 300  # 5분으로 늘림
        self.goal_marker_pub.publish(marker)

    # ==================== Safety & Rules ====================

    def _check_safety(self):
        """Check for collision and apply emergency stop if needed."""
        if not self.is_navigating:
            return

        if self.min_obstacle_dist < Config.COLLISION_STOP_DIST:
            self.get_logger().warn(
                f'EMERGENCY STOP! Obstacle at {self.min_obstacle_dist:.2f}m'
            )
            self._emergency_stop()

    def _check_rules(self):
        """Evaluate driving rules."""
        if not self.is_navigating:
            return

        # Check timeout
        if self.nav_start_time:
            elapsed = time.time() - self.nav_start_time
            if elapsed > Config.NAVIGATION_TIMEOUT:
                self.get_logger().warn('Navigation timeout, cancelling...')
                self._cancel_goal()
                return

        # Evaluate stop rules
        obstacle_in_path = self.min_obstacle_dist < Config.COLLISION_WARN_DIST
        action = self.rule_engine.evaluate_stop_rules(
            self.min_obstacle_dist, obstacle_in_path
        )

        if action.action_type == 'stop':
            self.get_logger().info(action.message)
            # Nav2 handles stopping, we just log

    def _emergency_stop(self):
        """Execute emergency stop by cancelling Nav2 goal."""
        self._cancel_goal()

    def _cancel_goal(self):
        """Cancel current navigation goal."""
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
        self.is_navigating = False
        self.current_goal = None

    def _log_status(self):
        """Log navigation status periodically."""
        success_rate = (
            (self.goals_succeeded / self.goals_attempted * 100)
            if self.goals_attempted > 0 else 0
        )

        status = "Navigating" if self.is_navigating else "Idle"
        goal_str = f"to ({self.current_goal[0]:.1f}, {self.current_goal[1]:.1f})" if self.current_goal else ""

        self.get_logger().info(
            f'Status: {status} {goal_str} | '
            f'Goals: {self.goals_succeeded}/{self.goals_attempted} ({success_rate:.0f}%)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f'\n=== Final Stats ===\n'
            f'Goals: {node.goals_succeeded}/{node.goals_attempted}'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


## 엔트리 포인트 ; 메인함수 호출
if __name__ == '__main__':
    main()
