#!/usr/bin/env python3
"""
Color Tracker Node: HSV-based color blob detection for following targets.

This node replaces MediaPipe-based detection with deterministic color tracking,
which is reliable in simulation environments with controlled lighting.

Subscribes: /camera/image_raw
Publishes:  /target_vector (TargetVector)
            /color_tracker/debug_image (Image)
            /color_tracker/mask (Image)

Control Logic:
- Blob size (area) determines proximity: large blob = close, small blob = far
- Normalized distance: 0.0 = very close (STOP), 1.0 = far (FOLLOW)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import numpy as np

from tb3_warehouse.msg import TargetVector


def imgmsg_to_cv2(msg):
    """Convert ROS Image message to OpenCV image."""
    dtype = np.uint8
    if msg.encoding == 'rgb8':
        channels = 3
    elif msg.encoding == 'bgr8':
        channels = 3
    elif msg.encoding == 'mono8':
        channels = 1
    elif msg.encoding in ('rgba8', 'bgra8'):
        channels = 4
    else:
        channels = 3

    img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)

    if msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    return img


def cv2_to_imgmsg(cv_image, encoding='bgr8'):
    """Convert OpenCV image to ROS Image message."""
    msg = Image()
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = False
    if len(cv_image.shape) > 2:
        msg.step = cv_image.shape[1] * cv_image.shape[2]
    else:
        msg.step = cv_image.shape[1]
    msg.data = cv_image.tobytes()
    return msg


class ColorTrackerNode(Node):
    """
    Color-based target tracking using HSV color space.

    Detects bright green objects (like safety vests) and computes:
    - Centroid position (x, y) normalized to [-1, 1]
    - Proximity based on blob area (larger = closer)
    """

    def __init__(self):
        super().__init__('color_tracker_node')

        # =================================================================
        # HSV Color Range Parameters (Neon Green)
        # =================================================================
        # Green in HSV: Hue ~35-85 (OpenCV uses 0-179 for Hue)
        # High saturation and value for bright/neon colors
        self.declare_parameter('hue_min', 35)      # Lower green hue bound
        self.declare_parameter('hue_max', 85)      # Upper green hue bound
        self.declare_parameter('sat_min', 100)     # Minimum saturation
        self.declare_parameter('sat_max', 255)     # Maximum saturation
        self.declare_parameter('val_min', 100)     # Minimum value (brightness)
        self.declare_parameter('val_max', 255)     # Maximum value

        # =================================================================
        # Detection Parameters
        # =================================================================
        self.declare_parameter('min_blob_area', 500)       # Minimum pixels to consider valid
        self.declare_parameter('max_blob_area', 100000)    # Maximum pixels (filter noise)
        self.declare_parameter('blur_kernel', 5)           # Gaussian blur kernel size
        self.declare_parameter('morph_kernel', 5)          # Morphological ops kernel size

        # =================================================================
        # Proximity Thresholds (blob area -> distance mapping)
        # =================================================================
        # These define the blob area ranges for distance estimation
        # Calibrate based on your camera and target size
        self.declare_parameter('close_area', 15000)   # Area when target is "close" (stop)
        self.declare_parameter('far_area', 1000)      # Area when target is "far" (full speed)

        # =================================================================
        # Topic Configuration
        # =================================================================
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('publish_debug', True)

        # Get parameters
        self.hue_min = self.get_parameter('hue_min').value
        self.hue_max = self.get_parameter('hue_max').value
        self.sat_min = self.get_parameter('sat_min').value
        self.sat_max = self.get_parameter('sat_max').value
        self.val_min = self.get_parameter('val_min').value
        self.val_max = self.get_parameter('val_max').value

        self.min_blob_area = self.get_parameter('min_blob_area').value
        self.max_blob_area = self.get_parameter('max_blob_area').value
        self.blur_kernel = self.get_parameter('blur_kernel').value
        self.morph_kernel = self.get_parameter('morph_kernel').value

        self.close_area = self.get_parameter('close_area').value
        self.far_area = self.get_parameter('far_area').value

        image_topic = self.get_parameter('image_topic').value
        self.publish_debug = self.get_parameter('publish_debug').value

        # HSV bounds as numpy arrays
        self.hsv_lower = np.array([self.hue_min, self.sat_min, self.val_min])
        self.hsv_upper = np.array([self.hue_max, self.sat_max, self.val_max])

        # Morphological kernel
        self.kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)

        # Publishers
        self.target_pub = self.create_publisher(TargetVector, '/target_vector', 10)
        self.detected_pub = self.create_publisher(Bool, '/target_detected', 10)

        if self.publish_debug:
            self.debug_img_pub = self.create_publisher(Image, '/color_tracker/debug_image', 10)
            self.mask_pub = self.create_publisher(Image, '/color_tracker/mask', 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # Tracking state
        self.frames_detected = 0
        self.frames_lost = 0
        self.DETECTION_THRESHOLD = 3  # Frames to confirm detection
        self.LOST_THRESHOLD = 10       # Frames before declaring lost

        self.get_logger().info(
            f"Color Tracker initialized\n"
            f"  HSV Range: H[{self.hue_min}-{self.hue_max}] "
            f"S[{self.sat_min}-{self.sat_max}] V[{self.val_min}-{self.val_max}]\n"
            f"  Blob Area: [{self.min_blob_area} - {self.max_blob_area}]\n"
            f"  Proximity: close={self.close_area}, far={self.far_area}\n"
            f"  Subscribed to: {image_topic}"
        )

    def image_callback(self, msg: Image):
        """Process incoming image and detect color blob."""
        try:
            cv_image = imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        if len(cv_image.shape) < 3:
            self.get_logger().warn("Received grayscale image, skipping")
            return

        h, w, _ = cv_image.shape

        # Step 1: Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(cv_image, (self.blur_kernel, self.blur_kernel), 0)

        # Step 2: Convert to HSV color space
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Step 3: Create mask for target color
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Step 4: Morphological operations to clean up mask
        # Opening removes small noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        # Closing fills small holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        # Dilate slightly to connect nearby regions
        mask = cv2.dilate(mask, self.kernel, iterations=1)

        # Step 5: Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Prepare output message
        target_msg = TargetVector()
        target_msg.detected = False
        target_msg.x = 0.0
        target_msg.y = 0.0
        target_msg.distance = 0.0

        detected_msg = Bool()
        detected_msg.data = False

        best_contour = None
        best_area = 0

        # Step 6: Find largest valid contour
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_blob_area < area < self.max_blob_area:
                if area > best_area:
                    best_area = area
                    best_contour = contour

        # Step 7: Process best contour if found
        if best_contour is not None:
            # Compute centroid using moments
            M = cv2.moments(best_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Normalize coordinates to [-1, 1] range
                # x: -1 = left edge, 0 = center, +1 = right edge
                # y: -1 = top edge, 0 = center, +1 = bottom edge
                target_msg.x = (cx / w - 0.5) * 2.0
                target_msg.y = (cy / h - 0.5) * 2.0

                # Compute proximity from blob area
                # Large area = close (low distance value)
                # Small area = far (high distance value)
                target_msg.distance = self._area_to_distance(best_area)

                # Update detection state
                self.frames_detected += 1
                self.frames_lost = 0

                if self.frames_detected >= self.DETECTION_THRESHOLD:
                    target_msg.detected = True
                    detected_msg.data = True

                # Debug visualization
                if self.publish_debug:
                    debug_img = self._draw_debug(
                        cv_image.copy(), best_contour, cx, cy,
                        best_area, target_msg
                    )
                    self.debug_img_pub.publish(cv2_to_imgmsg(debug_img, 'bgr8'))
        else:
            # No detection
            self.frames_lost += 1
            self.frames_detected = 0

            if self.publish_debug:
                debug_img = self._draw_no_detection(cv_image.copy())
                self.debug_img_pub.publish(cv2_to_imgmsg(debug_img, 'bgr8'))

        # Publish results
        self.target_pub.publish(target_msg)
        self.detected_pub.publish(detected_msg)

        # Publish mask for debugging
        if self.publish_debug:
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            self.mask_pub.publish(cv2_to_imgmsg(mask_color, 'bgr8'))

    def _area_to_distance(self, area: float) -> float:
        """
        Convert blob area to normalized distance value.

        Returns:
            float: 0.0 = very close (STOP), 1.0 = far (FOLLOW at full speed)

        The mapping is:
            area >= close_area -> distance = 0.0 (stop)
            area <= far_area   -> distance = 1.0 (full speed)
            in between         -> linear interpolation
        """
        if area >= self.close_area:
            return 0.0
        elif area <= self.far_area:
            return 1.0
        else:
            # Linear interpolation
            ratio = (self.close_area - area) / (self.close_area - self.far_area)
            return float(np.clip(ratio, 0.0, 1.0))

    def _draw_debug(self, image, contour, cx, cy, area, target_msg):
        """Draw debug visualization with detection info."""
        # Draw contour
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)

        # Draw bounding box
        x, y, bw, bh = cv2.boundingRect(contour)
        cv2.rectangle(image, (x, y), (x + bw, y + bh), (255, 0, 0), 2)

        # Draw centroid crosshair
        cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)
        cv2.line(image, (cx - 20, cy), (cx + 20, cy), (0, 0, 255), 2)
        cv2.line(image, (cx, cy - 20), (cx, cy + 20), (0, 0, 255), 2)

        # Draw center line (reference)
        h, w = image.shape[:2]
        cv2.line(image, (w // 2, 0), (w // 2, h), (128, 128, 128), 1)

        # Status text
        status = "DETECTED" if target_msg.detected else "ACQUIRING..."
        color = (0, 255, 0) if target_msg.detected else (0, 255, 255)
        cv2.putText(image, f"Status: {status}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # Position info
        cv2.putText(image, f"X: {target_msg.x:+.2f} (left/right)", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(image, f"Y: {target_msg.y:+.2f} (up/down)", (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Proximity info
        dist = target_msg.distance
        if dist < 0.3:
            prox_text = "CLOSE (STOP)"
            prox_color = (0, 0, 255)
        elif dist < 0.7:
            prox_text = "MEDIUM (SLOW)"
            prox_color = (0, 255, 255)
        else:
            prox_text = "FAR (FOLLOW)"
            prox_color = (0, 255, 0)

        cv2.putText(image, f"Distance: {dist:.2f} - {prox_text}", (10, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, prox_color, 2)

        # Area info
        cv2.putText(image, f"Blob Area: {int(area)} px", (10, 145),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

        # Draw proximity bar
        bar_x, bar_y, bar_w, bar_h = 10, 160, 200, 20
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (100, 100, 100), -1)
        fill_w = int(bar_w * (1.0 - dist))  # Invert so full = close
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), prox_color, -1)
        cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (255, 255, 255), 1)
        cv2.putText(image, "CLOSE", (bar_x, bar_y + bar_h + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(image, "FAR", (bar_x + bar_w - 25, bar_y + bar_h + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        return image

    def _draw_no_detection(self, image):
        """Draw debug visualization when no target detected."""
        cv2.putText(image, "Status: NO TARGET", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(image, f"Frames lost: {self.frames_lost}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

        # Draw center crosshair
        h, w = image.shape[:2]
        cv2.line(image, (w // 2 - 30, h // 2), (w // 2 + 30, h // 2), (128, 128, 128), 1)
        cv2.line(image, (w // 2, h // 2 - 30), (w // 2, h // 2 + 30), (128, 128, 128), 1)

        return image


def main(args=None):
    rclpy.init(args=args)
    node = ColorTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
