#!/usr/bin/env python3
"""
Control Node: PID-based person following.
Subscribes: /target_vector, /gesture_cmd
Publishes:  /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tb3_warehouse.msg import TargetVector, GestureCmd


class PIDController:
    """Simple PID controller with anti-windup."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float, output_max: float, windup_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.windup_limit = windup_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error: float, current_time: float) -> float:
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0

        dt = current_time - self.prev_time
        if dt <= 0.0:
            return 0.0

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(-self.windup_limit, min(self.windup_limit, self.integral))
        i_term = self.ki * self.integral

        # Derivative
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Update state
        self.prev_error = error
        self.prev_time = current_time

        # Compute output with saturation
        output = p_term + i_term + d_term
        return max(self.output_min, min(self.output_max, output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Parameters - PID gains
        self.declare_parameter('angular_kp', 1.2)
        self.declare_parameter('angular_ki', 0.01)
        self.declare_parameter('angular_kd', 0.3)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('linear_ki', 0.0)
        self.declare_parameter('linear_kd', 0.1)

        # Parameters - velocity limits
        self.declare_parameter('max_linear_vel', 0.22)   # TB3 max: 0.22 m/s
        self.declare_parameter('max_angular_vel', 1.5)   # TB3 max: 2.84 rad/s
        self.declare_parameter('min_linear_vel', 0.0)

        # Parameters - following behavior
        self.declare_parameter('target_distance', 0.4)   # Desired normalized distance
        self.declare_parameter('deadzone_x', 0.05)       # Angular deadzone
        self.declare_parameter('deadzone_dist', 0.05)    # Distance deadzone
        self.declare_parameter('lost_target_timeout', 2.0)

        # Get parameters
        ang_kp = self.get_parameter('angular_kp').value
        ang_ki = self.get_parameter('angular_ki').value
        ang_kd = self.get_parameter('angular_kd').value
        lin_kp = self.get_parameter('linear_kp').value
        lin_ki = self.get_parameter('linear_ki').value
        lin_kd = self.get_parameter('linear_kd').value

        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.min_linear = self.get_parameter('min_linear_vel').value
        self.target_distance = self.get_parameter('target_distance').value
        self.deadzone_x = self.get_parameter('deadzone_x').value
        self.deadzone_dist = self.get_parameter('deadzone_dist').value
        self.lost_timeout = self.get_parameter('lost_target_timeout').value

        # PID controllers
        self.angular_pid = PIDController(
            ang_kp, ang_ki, ang_kd,
            -self.max_angular, self.max_angular
        )
        self.linear_pid = PIDController(
            lin_kp, lin_ki, lin_kd,
            self.min_linear, self.max_linear
        )

        # State
        self.following_enabled = False
        self.last_target_time = None
        self.current_target = None

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.target_sub = self.create_subscription(
            TargetVector, '/target_vector', self.target_callback, 10)
        self.gesture_sub = self.create_subscription(
            GestureCmd, '/gesture_cmd', self.gesture_callback, 10)

        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Control node initialized. Waiting for START gesture...")

    def gesture_callback(self, msg: GestureCmd):
        """Handle gesture commands to enable/disable following."""
        if not msg.active:
            return

        if msg.gesture == "START" and not self.following_enabled:
            self.following_enabled = True
            self.angular_pid.reset()
            self.linear_pid.reset()
            self.get_logger().info("Following ENABLED (START gesture)")

        elif msg.gesture == "STOP" and self.following_enabled:
            self.following_enabled = False
            self.stop_robot()
            self.get_logger().info("Following DISABLED (STOP gesture)")

    def target_callback(self, msg: TargetVector):
        """Store latest target position."""
        self.current_target = msg
        if msg.detected:
            self.last_target_time = self.get_clock().now()

    def control_loop(self):
        """Main control loop - compute and publish velocity commands."""
        cmd = Twist()
        now = self.get_clock().now()
        current_time = now.nanoseconds / 1e9

        # Check if following is enabled
        if not self.following_enabled:
            self.cmd_pub.publish(cmd)
            return

        # Check for target timeout
        if self.last_target_time is None:
            self.cmd_pub.publish(cmd)
            return

        time_since_target = (now - self.last_target_time).nanoseconds / 1e9
        if time_since_target > self.lost_timeout:
            self.get_logger().warn("Target lost - stopping", throttle_duration_sec=2.0)
            self.stop_robot()
            return

        # Check if we have valid target
        if self.current_target is None or not self.current_target.detected:
            self.cmd_pub.publish(cmd)
            return

        target = self.current_target

        # Angular control: minimize x offset (center person in frame)
        # Negative x means person is to the left, robot should turn left (positive angular)
        angular_error = -target.x  # Invert: left in image = turn left

        if abs(angular_error) > self.deadzone_x:
            cmd.angular.z = self.angular_pid.compute(angular_error, current_time)
        else:
            cmd.angular.z = 0.0

        # Linear control: maintain target distance
        # Higher distance value = closer to camera = move backward (or stop)
        # Lower distance value = farther = move forward
        distance_error = self.target_distance - target.distance

        if abs(distance_error) > self.deadzone_dist:
            # Only move forward if person is far, stop/slow if too close
            linear_vel = self.linear_pid.compute(distance_error, current_time)
            # Prevent backward motion (let safety node handle that)
            cmd.linear.x = max(0.0, linear_vel)
        else:
            cmd.linear.x = 0.0

        # Reduce linear speed during sharp turns
        turn_factor = 1.0 - min(abs(cmd.angular.z) / self.max_angular, 0.7)
        cmd.linear.x *= turn_factor

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Publish zero velocity."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
