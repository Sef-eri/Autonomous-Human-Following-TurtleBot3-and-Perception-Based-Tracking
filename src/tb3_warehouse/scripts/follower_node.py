#!/usr/bin/env python3
"""
Follower Node: Proximity-based target following controller.

This node replaces gesture-based control with proximity detection:
- FOLLOW: When target blob is small (far away)
- STOP: When target blob is large (close)

The control uses:
- Angular velocity to center the target in the camera frame
- Linear velocity proportional to distance (far = fast, close = slow/stop)

Subscribes: /target_vector (TargetVector)
Publishes:  /cmd_vel (Twist) or /cmd_vel_raw if safety node is used
            /follower/state (String)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from tb3_warehouse.msg import TargetVector


class FollowerState:
    """Enumeration of follower states."""
    IDLE = "IDLE"           # No target, waiting
    SEARCHING = "SEARCHING"  # Lost target, searching
    FOLLOWING = "FOLLOWING"  # Target found, following
    STOPPING = "STOPPING"    # Target too close, stopping


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
            return self.kp * error  # Initial proportional only

        dt = current_time - self.prev_time
        if dt <= 0.0:
            return self.kp * error

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


class FollowerNode(Node):
    """
    Proximity-based target follower.

    Control behavior:
    - When target.distance is high (far): Move forward to approach
    - When target.distance is low (close): Slow down or stop
    - Angular control keeps target centered in frame
    """

    def __init__(self):
        super().__init__('follower_node')

        # =================================================================
        # Velocity Limits
        # =================================================================
        self.declare_parameter('max_linear_vel', 0.7)    # TB3 max: 0.22 m/s
        self.declare_parameter('max_angular_vel', 1.5)    # TB3 max: 2.84 rad/s
        self.declare_parameter('min_linear_vel', 0.05)    # Minimum movement speed

        # =================================================================
        # PID Gains
        # =================================================================
        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('angular_ki', 0.01)
        self.declare_parameter('angular_kd', 0.2)

        # =================================================================
        # Following Behavior
        # =================================================================
        self.declare_parameter('stop_distance', 0.2)      # Distance below which to stop
        self.declare_parameter('slow_distance', 0.5)      # Distance below which to slow
        self.declare_parameter('deadzone_x', 0.05)        # Angular deadzone (no turn if small)
        self.declare_parameter('lost_target_timeout', 2.0)  # Seconds before "lost"
        self.declare_parameter('search_angular_vel', 0.4)   # Spin speed when searching

        # =================================================================
        # Auto-start (no gesture required)
        # =================================================================
        self.declare_parameter('auto_start', True)        # Start following immediately

        # Get parameters
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.min_linear = self.get_parameter('min_linear_vel').value

        ang_kp = self.get_parameter('angular_kp').value
        ang_ki = self.get_parameter('angular_ki').value
        ang_kd = self.get_parameter('angular_kd').value

        self.stop_distance = self.get_parameter('stop_distance').value
        self.slow_distance = self.get_parameter('slow_distance').value
        self.deadzone_x = self.get_parameter('deadzone_x').value
        self.lost_timeout = self.get_parameter('lost_target_timeout').value
        self.search_angular = self.get_parameter('search_angular_vel').value
        self.auto_start = self.get_parameter('auto_start').value

        # Angular PID controller
        self.angular_pid = PIDController(
            ang_kp, ang_ki, ang_kd,
            -self.max_angular, self.max_angular
        )

        # State
        self.state = FollowerState.IDLE if not self.auto_start else FollowerState.SEARCHING
        self.enabled = self.auto_start
        self.current_target = None
        self.last_target_time = None
        self.search_direction = 1  # 1 = left, -1 = right

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/follower/state', 10)

        # Subscribers
        self.target_sub = self.create_subscription(
            TargetVector, '/target_vector', self.target_callback, 10)

        # Optional: Enable/disable via topic
        self.enable_sub = self.create_subscription(
            Bool, '/follower/enable', self.enable_callback, 10)

        # Control loop timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # State publishing timer (2 Hz)
        self.state_timer = self.create_timer(0.5, self.publish_state)

        self.get_logger().info(
            f"Follower node initialized\n"
            f"  Max velocities: linear={self.max_linear}, angular={self.max_angular}\n"
            f"  Stop distance: {self.stop_distance}, Slow distance: {self.slow_distance}\n"
            f"  Auto-start: {self.auto_start}"
        )

    def enable_callback(self, msg: Bool):
        """Handle enable/disable commands."""
        if msg.data and not self.enabled:
            self.enabled = True
            self.state = FollowerState.SEARCHING
            self.angular_pid.reset()
            self.get_logger().info("Follower ENABLED")
        elif not msg.data and self.enabled:
            self.enabled = False
            self.state = FollowerState.IDLE
            self.stop_robot()
            self.get_logger().info("Follower DISABLED")

    def target_callback(self, msg: TargetVector):
        """Store latest target detection."""
        self.current_target = msg
        if msg.detected:
            self.last_target_time = self.get_clock().now()

    def control_loop(self):
        """Main control loop."""
        cmd = Twist()
        now = self.get_clock().now()
        current_time = now.nanoseconds / 1e9

        # Check if enabled
        if not self.enabled:
            self.state = FollowerState.IDLE
            self.cmd_pub.publish(cmd)
            return

        # Check for target timeout
        target_lost = False
        if self.last_target_time is not None:
            time_since_target = (now - self.last_target_time).nanoseconds / 1e9
            target_lost = time_since_target > self.lost_timeout

        # State machine
        if self.current_target is None or not self.current_target.detected or target_lost:
            # No target - search behavior
            self.state = FollowerState.SEARCHING
            cmd = self._search_behavior()
        else:
            # Target detected - following behavior
            cmd = self._follow_behavior(current_time)

        self.cmd_pub.publish(cmd)

    def _search_behavior(self) -> Twist:
        """Rotate in place to search for target."""
        cmd = Twist()
        cmd.angular.z = self.search_angular * self.search_direction

        # Could add logic to reverse direction after timeout
        return cmd

    def _follow_behavior(self, current_time: float) -> Twist:
        """Follow the detected target based on proximity."""
        cmd = Twist()
        target = self.current_target

        # Distance-based behavior
        dist = target.distance  # 0.0 = close, 1.0 = far

        if dist < self.stop_distance:
            # Too close - stop forward motion
            self.state = FollowerState.STOPPING
            cmd.linear.x = 0.0
        elif dist < self.slow_distance:
            # Getting close - slow down proportionally
            self.state = FollowerState.FOLLOWING
            # Linear interpolation between stop and slow distances
            speed_factor = (dist - self.stop_distance) / (self.slow_distance - self.stop_distance)
            cmd.linear.x = self.min_linear + (self.max_linear - self.min_linear) * speed_factor
        else:
            # Far away - full speed
            self.state = FollowerState.FOLLOWING
            # Speed proportional to distance (faster when farther)
            speed_factor = min(dist, 1.0)
            cmd.linear.x = self.min_linear + (self.max_linear - self.min_linear) * speed_factor

        # Angular control: center the target
        # target.x: -1 = left side of image, +1 = right side
        # To center, if target is on left (negative x), turn left (positive angular)
        angular_error = -target.x  # Invert: target on left -> turn left

        if abs(angular_error) > self.deadzone_x:
            cmd.angular.z = self.angular_pid.compute(angular_error, current_time)
        else:
            cmd.angular.z = 0.0

        # Reduce linear speed during sharp turns
        turn_factor = 1.0 - min(abs(cmd.angular.z) / self.max_angular, 0.7)
        cmd.linear.x *= turn_factor

        # Update search direction based on last known target position
        if target.x < 0:
            self.search_direction = 1  # Target was on left, search left
        else:
            self.search_direction = -1  # Target was on right, search right

        return cmd

    def stop_robot(self):
        """Publish zero velocity."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def publish_state(self):
        """Publish current state for monitoring."""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
