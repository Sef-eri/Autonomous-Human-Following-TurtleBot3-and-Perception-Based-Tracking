#!/usr/bin/env python3
"""
Vision Node: Person detection and gesture recognition using MediaPipe.
Subscribes: /camera/image_raw
Publishes:  /target_vector (TargetVector), /gesture_cmd (GestureCmd)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    print("WARNING: MediaPipe not installed. Using fallback detection.")

from tb3_warehouse.msg import TargetVector, GestureCmd


def imgmsg_to_cv2(msg):
    """Convert ROS Image message to OpenCV image without cv_bridge."""
    dtype = np.uint8
    if msg.encoding == 'rgb8':
        channels = 3
    elif msg.encoding == 'bgr8':
        channels = 3
    elif msg.encoding == 'mono8':
        channels = 1
    elif msg.encoding == 'rgba8' or msg.encoding == 'bgra8':
        channels = 4
    else:
        channels = 3  # Default assumption

    img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

    if msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    return img


def cv2_to_imgmsg(cv_image, encoding='bgr8'):
    """Convert OpenCV image to ROS Image message without cv_bridge."""
    msg = Image()
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = cv_image.shape[1] * cv_image.shape[2] if len(cv_image.shape) > 2 else cv_image.shape[1]
    msg.data = cv_image.tobytes()
    return msg


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('tracking_confidence', 0.5)
        self.declare_parameter('publish_debug_image', True)

        image_topic = self.get_parameter('image_topic').value
        det_conf = self.get_parameter('detection_confidence').value
        track_conf = self.get_parameter('tracking_confidence').value
        self.publish_debug = self.get_parameter('publish_debug_image').value

        # Publishers
        self.target_pub = self.create_publisher(TargetVector, '/target_vector', 10)
        self.gesture_pub = self.create_publisher(GestureCmd, '/gesture_cmd', 10)
        self.debug_img_pub = self.create_publisher(Image, '/vision/debug_image', 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # MediaPipe setup
        if MEDIAPIPE_AVAILABLE:
            self.mp_pose = mp.solutions.pose
            self.mp_hands = mp.solutions.hands
            self.mp_draw = mp.solutions.drawing_utils

            self.pose = self.mp_pose.Pose(
                static_image_mode=False,
                model_complexity=1,
                min_detection_confidence=det_conf,
                min_tracking_confidence=track_conf
            )
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=2,
                min_detection_confidence=det_conf,
                min_tracking_confidence=track_conf
            )
        else:
            self.pose = None
            self.hands = None

        # Gesture state
        self.prev_gesture = "NONE"
        self.gesture_hold_frames = 0
        self.GESTURE_THRESHOLD = 5  # Frames to confirm gesture

        self.get_logger().info(f"Vision node initialized. Subscribed to: {image_topic}")
        if MEDIAPIPE_AVAILABLE:
            self.get_logger().info("MediaPipe enabled for pose/gesture detection")
        else:
            self.get_logger().warn("MediaPipe not available - using color fallback")

    def image_callback(self, msg: Image):
        try:
            cv_image = imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        if len(cv_image.shape) < 3:
            self.get_logger().warn("Received grayscale image, skipping")
            return

        h, w, _ = cv_image.shape
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        target_msg = TargetVector()
        gesture_msg = GestureCmd()
        gesture_msg.gesture = "NONE"
        gesture_msg.active = False
        gesture_msg.confidence = 0.0

        if MEDIAPIPE_AVAILABLE and self.pose is not None:
            # Process pose for person tracking
            pose_results = self.pose.process(rgb_image)
            target_msg = self.process_pose(pose_results, w, h)

            # Process hands for gesture recognition
            hand_results = self.hands.process(rgb_image)
            gesture_msg = self.process_gestures(hand_results)

            # Draw debug visualization
            if self.publish_debug:
                debug_image = self.draw_debug(cv_image, pose_results, hand_results, target_msg, gesture_msg)
                self.debug_img_pub.publish(cv2_to_imgmsg(debug_image, 'bgr8'))
        else:
            # Fallback: simple color-based detection (blue shirt)
            target_msg = self.fallback_detection(cv_image, w, h)
            if self.publish_debug:
                self.debug_img_pub.publish(cv2_to_imgmsg(cv_image, 'bgr8'))

        # Publish
        self.target_pub.publish(target_msg)
        self.gesture_pub.publish(gesture_msg)

    def process_pose(self, results, img_w: int, img_h: int) -> TargetVector:
        """Extract person center from pose landmarks."""
        msg = TargetVector()
        msg.detected = False
        msg.x = 0.0
        msg.y = 0.0
        msg.distance = 0.0

        if not results.pose_landmarks:
            return msg

        landmarks = results.pose_landmarks.landmark

        # Use torso landmarks for stable tracking
        torso_indices = [11, 12, 23, 24]
        xs, ys = [], []

        for idx in torso_indices:
            lm = landmarks[idx]
            if lm.visibility > 0.5:
                xs.append(lm.x)
                ys.append(lm.y)

        if len(xs) < 2:
            nose = landmarks[0]
            if nose.visibility > 0.5:
                xs, ys = [nose.x], [nose.y]

        if xs:
            cx = np.mean(xs)
            cy = np.mean(ys)

            msg.x = float((cx - 0.5) * 2.0)
            msg.y = float((cy - 0.5) * 2.0)

            if len(xs) >= 2:
                shoulder_width = abs(landmarks[11].x - landmarks[12].x)
                msg.distance = float(min(shoulder_width * 3.0, 1.0))
            else:
                msg.distance = 0.5

            msg.detected = True

        return msg

    def process_gestures(self, results) -> GestureCmd:
        """Detect hand gestures for START/STOP commands."""
        msg = GestureCmd()
        msg.gesture = "NONE"
        msg.active = False
        msg.confidence = 0.0

        if not results.multi_hand_landmarks:
            self.gesture_hold_frames = 0
            return msg

        for hand_landmarks, handedness in zip(
            results.multi_hand_landmarks,
            results.multi_handedness
        ):
            gesture, confidence = self.classify_gesture(hand_landmarks.landmark)

            if gesture != "NONE":
                if gesture == self.prev_gesture:
                    self.gesture_hold_frames += 1
                else:
                    self.gesture_hold_frames = 1
                    self.prev_gesture = gesture

                if self.gesture_hold_frames >= self.GESTURE_THRESHOLD:
                    msg.gesture = gesture
                    msg.active = True
                    msg.confidence = confidence
                    return msg

        return msg

    def classify_gesture(self, landmarks) -> tuple:
        """Classify hand gesture based on finger positions."""
        tips = [4, 8, 12, 16, 20]
        pips = [3, 6, 10, 14, 18]

        fingers_up = []

        thumb_tip = landmarks[4]
        thumb_ip = landmarks[3]
        thumb_up = thumb_tip.y < thumb_ip.y - 0.05

        for tip_idx, pip_idx in zip(tips[1:], pips[1:]):
            tip = landmarks[tip_idx]
            pip = landmarks[pip_idx]
            fingers_up.append(tip.y < pip.y - 0.02)

        num_fingers_extended = sum(fingers_up)

        if num_fingers_extended >= 4:
            return ("STOP", 0.9)

        if thumb_up and num_fingers_extended <= 1:
            return ("START", 0.85)

        return ("NONE", 0.0)

    def fallback_detection(self, image, img_w: int, img_h: int) -> TargetVector:
        """Fallback detection using color segmentation (blue clothing)."""
        msg = TargetVector()
        msg.detected = False

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 500:
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    msg.x = float((cx / img_w - 0.5) * 2.0)
                    msg.y = float((cy / img_h - 0.5) * 2.0)
                    msg.distance = float(min(area / (img_w * img_h) * 10, 1.0))
                    msg.detected = True

        return msg

    def draw_debug(self, image, pose_results, hand_results, target: TargetVector, gesture: GestureCmd):
        """Draw debug visualization."""
        debug_img = image.copy()
        h, w, _ = debug_img.shape

        if MEDIAPIPE_AVAILABLE:
            if pose_results.pose_landmarks:
                self.mp_draw.draw_landmarks(
                    debug_img,
                    pose_results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    connection_drawing_spec=self.mp_draw.DrawingSpec(color=(0, 255, 0), thickness=2)
                )

            if hand_results.multi_hand_landmarks:
                for hand_lm in hand_results.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(
                        debug_img,
                        hand_lm,
                        self.mp_hands.HAND_CONNECTIONS,
                        landmark_drawing_spec=self.mp_draw.DrawingSpec(color=(255, 0, 0), thickness=2, circle_radius=2),
                        connection_drawing_spec=self.mp_draw.DrawingSpec(color=(255, 0, 0), thickness=2)
                    )

        if target.detected:
            cx = int((target.x / 2.0 + 0.5) * w)
            cy = int((target.y / 2.0 + 0.5) * h)
            cv2.circle(debug_img, (cx, cy), 20, (0, 0, 255), 3)
            cv2.line(debug_img, (cx - 30, cy), (cx + 30, cy), (0, 0, 255), 2)
            cv2.line(debug_img, (cx, cy - 30), (cx, cy + 30), (0, 0, 255), 2)

        status = f"Target: {'DETECTED' if target.detected else 'NONE'} | Gesture: {gesture.gesture}"
        cv2.putText(debug_img, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if target.detected:
            pos_text = f"X: {target.x:.2f} Y: {target.y:.2f} Dist: {target.distance:.2f}"
            cv2.putText(debug_img, pos_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return debug_img

    def destroy_node(self):
        if MEDIAPIPE_AVAILABLE and self.pose is not None:
            self.pose.close()
            self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
