#!/usr/bin/env python3
"""
Behavior Tree Node: State machine for warehouse assistant.
States: IDLE → FOLLOW → SEARCH → STOP

Subscribes: /gesture_cmd_raw, /target_vector, /obstacle_detected
Publishes:  /behavior_state, /gesture_cmd (to control node)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from tb3_warehouse.msg import TargetVector, GestureCmd


class BehaviorTreeNode(Node):
    def __init__(self):
        super().__init__('behavior_tree_node')

        # Parameters
        self.declare_parameter('tick_rate', 10.0)
        self.declare_parameter('target_lost_timeout', 3.0)
        self.declare_parameter('search_angular_vel', 0.5)

        self.tick_rate = self.get_parameter('tick_rate').value
        self.target_timeout = self.get_parameter('target_lost_timeout').value
        self.search_angular = self.get_parameter('search_angular_vel').value

        # State
        self.current_state = "IDLE"
        self.gesture_cmd = None
        self.target = None
        self.obstacle_detected = False
        self.last_target_time = None
        self.following_enabled = False
        self.search_direction = 1.0

        # Publishers
        self.state_pub = self.create_publisher(String, '/behavior_state', 10)
        self.gesture_internal_pub = self.create_publisher(GestureCmd, '/gesture_cmd', 10)

        # Subscribers
        self.gesture_sub = self.create_subscription(
            GestureCmd, '/gesture_cmd_raw', self.gesture_callback, 10)
        self.target_sub = self.create_subscription(
            TargetVector, '/target_vector', self.target_callback, 10)
        self.obstacle_sub = self.create_subscription(
            Bool, '/obstacle_detected', self.obstacle_callback, 10)

        # Tick timer
        self.tick_timer = self.create_timer(1.0 / self.tick_rate, self.tick)

        self.get_logger().info("Behavior tree initialized. State: IDLE")
        self.publish_state()

    def gesture_callback(self, msg: GestureCmd):
        """Process incoming gesture commands."""
        if not msg.active:
            return

        if msg.gesture == "START" and not self.following_enabled:
            self.following_enabled = True
            self.set_state("SEARCH")  # Start in search mode
            self.forward_gesture("START")
            self.get_logger().info("START received - following enabled")

        elif msg.gesture == "STOP" and self.following_enabled:
            self.following_enabled = False
            self.set_state("IDLE")
            self.forward_gesture("STOP")
            self.get_logger().info("STOP received - returning to IDLE")

    def target_callback(self, msg: TargetVector):
        """Process target detection."""
        self.target = msg
        if msg.detected:
            self.last_target_time = self.get_clock().now()

    def obstacle_callback(self, msg: Bool):
        """Process obstacle detection."""
        self.obstacle_detected = msg.data

    def tick(self):
        """Main behavior tree tick - simple state machine."""

        # Priority 1: STOP gesture always works
        # (handled in gesture_callback)

        # Priority 2: Obstacle avoidance (handled by safety node)
        if self.obstacle_detected and self.following_enabled:
            if self.current_state != "OBSTACLE_AVOID":
                self.set_state("OBSTACLE_AVOID")
            return

        # If not following, stay in IDLE
        if not self.following_enabled:
            if self.current_state != "IDLE":
                self.set_state("IDLE")
            return

        # Following is enabled - check for target
        target_visible = self.target is not None and self.target.detected

        if target_visible:
            # Target visible - follow mode
            if self.current_state != "FOLLOW":
                self.set_state("FOLLOW")
        else:
            # No target - check if recently lost
            if self.last_target_time is not None:
                elapsed = (self.get_clock().now() - self.last_target_time).nanoseconds / 1e9
                if elapsed < self.target_timeout:
                    # Recently lost - search mode
                    if self.current_state != "SEARCH":
                        self.set_state("SEARCH")
                    return

            # No target ever seen or timeout - still search
            if self.current_state != "SEARCH":
                self.set_state("SEARCH")

    def set_state(self, new_state: str):
        """Update and publish state."""
        if self.current_state != new_state:
            self.get_logger().info(f"State: {self.current_state} → {new_state}")
            self.current_state = new_state
            self.publish_state()

    def publish_state(self):
        """Publish current state."""
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

    def forward_gesture(self, gesture: str):
        """Forward gesture to control node."""
        msg = GestureCmd()
        msg.gesture = gesture
        msg.active = True
        msg.confidence = 1.0
        self.gesture_internal_pub.publish(msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
