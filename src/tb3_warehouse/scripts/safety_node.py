#!/usr/bin/env python3
"""
Safety Node: LiDAR-based reactive obstacle avoidance.
Subscribes: /scan, /cmd_vel
Publishes:  /cmd_vel_safe (filtered velocity)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Parameters
        self.declare_parameter('stop_distance', 0.5)       # Emergency stop
        self.declare_parameter('slow_distance', 1.0)       # Start slowing
        self.declare_parameter('side_distance', 0.35)      # Side clearance
        self.declare_parameter('front_angle', 60.0)        # Front sector (degrees)
        self.declare_parameter('side_angle', 30.0)         # Side sector (degrees)
        self.declare_parameter('avoidance_angular_vel', 0.8)
        self.declare_parameter('min_linear_scale', 0.2)    # Min speed factor when obstacle detected
        self.declare_parameter('enable_avoidance', True)   # Enable steering around obstacles

        self.stop_dist = self.get_parameter('stop_distance').value
        self.slow_dist = self.get_parameter('slow_distance').value
        self.side_dist = self.get_parameter('side_distance').value
        self.front_angle = math.radians(self.get_parameter('front_angle').value)
        self.side_angle = math.radians(self.get_parameter('side_angle').value)
        self.avoid_angular = self.get_parameter('avoidance_angular_vel').value
        self.min_scale = self.get_parameter('min_linear_scale').value
        self.enable_avoidance = self.get_parameter('enable_avoidance').value

        # State
        self.latest_cmd = Twist()
        self.obstacle_detected = False
        self.emergency_stop = False

        # Sector distances
        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

        # Publishers
        self.cmd_safe_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # Safety loop (50 Hz)
        self.safety_timer = self.create_timer(0.02, self.safety_loop)

        self.get_logger().info(
            f"Safety node initialized. Stop: {self.stop_dist}m, Slow: {self.slow_dist}m"
        )

    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan and compute sector minimum distances."""
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        # Reset sector minimums
        front_min = float('inf')
        left_min = float('inf')
        right_min = float('inf')

        for i, distance in enumerate(msg.ranges):
            # Skip invalid readings
            if distance < msg.range_min or distance > msg.range_max:
                continue
            if math.isnan(distance) or math.isinf(distance):
                continue

            # Compute angle (0 = forward, positive = left, negative = right)
            angle = angle_min + i * angle_inc

            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # Categorize into sectors
            abs_angle = abs(angle)

            # Front sector
            if abs_angle < self.front_angle / 2:
                front_min = min(front_min, distance)

            # Left sector (front-left)
            if 0 < angle < (self.front_angle / 2 + self.side_angle):
                left_min = min(left_min, distance)

            # Right sector (front-right)
            if -(self.front_angle / 2 + self.side_angle) < angle < 0:
                right_min = min(right_min, distance)

        self.front_min = front_min
        self.left_min = left_min
        self.right_min = right_min

        # Update obstacle state
        self.emergency_stop = front_min < self.stop_dist
        self.obstacle_detected = front_min < self.slow_dist

        # Publish obstacle status
        obs_msg = Bool()
        obs_msg.data = self.obstacle_detected
        self.obstacle_pub.publish(obs_msg)

    def cmd_callback(self, msg: Twist):
        """Store latest velocity command from control node."""
        self.latest_cmd = msg

    def safety_loop(self):
        """Apply safety filtering to velocity commands."""
        cmd = Twist()
        input_cmd = self.latest_cmd

        # Emergency stop - obstacle too close
        if self.emergency_stop:
            # Allow rotation in place and backward motion
            if input_cmd.linear.x > 0:
                cmd.linear.x = 0.0
                self.get_logger().warn(
                    f"EMERGENCY STOP - obstacle at {self.front_min:.2f}m",
                    throttle_duration_sec=1.0
                )
            else:
                cmd.linear.x = input_cmd.linear.x  # Allow reverse

            # Compute avoidance turn if enabled
            if self.enable_avoidance and input_cmd.linear.x > 0:
                cmd.angular.z = self.compute_avoidance_turn()
            else:
                cmd.angular.z = input_cmd.angular.z

            self.cmd_safe_pub.publish(cmd)
            return

        # Slowdown zone - reduce speed proportionally
        if self.obstacle_detected:
            # Linear interpolation between stop and slow distances
            scale = (self.front_min - self.stop_dist) / (self.slow_dist - self.stop_dist)
            scale = max(self.min_scale, min(1.0, scale))

            cmd.linear.x = input_cmd.linear.x * scale

            # Add avoidance steering if enabled
            if self.enable_avoidance and input_cmd.linear.x > 0.05:
                avoidance_turn = self.compute_avoidance_turn() * (1.0 - scale)
                cmd.angular.z = input_cmd.angular.z + avoidance_turn
            else:
                cmd.angular.z = input_cmd.angular.z

            self.cmd_safe_pub.publish(cmd)
            return

        # No obstacle - pass through
        cmd.linear.x = input_cmd.linear.x
        cmd.angular.z = input_cmd.angular.z
        self.cmd_safe_pub.publish(cmd)

    def compute_avoidance_turn(self) -> float:
        """
        Compute avoidance steering direction.
        Turn away from the closer side.
        """
        # If left is closer, turn right (negative angular)
        # If right is closer, turn left (positive angular)
        if self.left_min < self.right_min:
            return -self.avoid_angular  # Turn right
        elif self.right_min < self.left_min:
            return self.avoid_angular   # Turn left
        else:
            # Equal or unknown - turn right by default
            return -self.avoid_angular

    def destroy_node(self):
        # Stop robot on shutdown
        cmd = Twist()
        self.cmd_safe_pub.publish(cmd)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
