#!/usr/bin/env python3
"""
vision_control_node.py
----------------------
ROS 2 Humble node that:
  1. Subscribes to /camera/image_raw
  2. Detects a target using OpenCV
  3. Calculates distance, angle, and required velocity
  4. Publishes Twist commands to /cmd_vel (read by arduino_bridge_node)

The camera acts as the feedback sensor — no data needs to come back from the Arduino.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionControlNode(Node):
    def __init__(self):
        super().__init__('vision_control')

        # ---------------------------------------------------------------
        # Camera intrinsics — update these after running camera calibration
        # ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025
        # ---------------------------------------------------------------
        self.FOCAL_LENGTH_PX      = 600.0   # pixels — from calibration
        self.KNOWN_TARGET_WIDTH_M = 0.10    # real-world width of target [m]
        self.IMAGE_WIDTH_PX       = 640     # horizontal resolution [px]
        self.IMAGE_HEIGHT_PX      = 480     # vertical resolution [px]

        # ---------------------------------------------------------------
        # Control parameters
        # ---------------------------------------------------------------
        self.TARGET_DISTANCE_M = 0.40   # desired stand-off distance [m]
        self.K_LINEAR          = 0.5    # proportional gain: distance error → linear speed
        self.K_ANGULAR         = 1.5    # proportional gain: angle error  → angular speed
        self.MAX_LINEAR_SPEED  = 0.5    # m/s  — clamp to safe value
        self.MAX_ANGULAR_SPEED = 1.0    # rad/s

        # ---------------------------------------------------------------
        # Target colour range in HSV — tune for your target
        # Default: green object
        # ---------------------------------------------------------------
        self.HSV_LOWER = np.array([35, 100, 100])
        self.HSV_UPPER = np.array([85, 255, 255])

        # Minimum contour area to be considered a valid detection [px²]
        self.MIN_CONTOUR_AREA = 500

        # ---------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------
        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Vision control node started.')
        self.get_logger().info(f'Target stand-off distance: {self.TARGET_DISTANCE_M} m')

    # -------------------------------------------------------------------
    # Geometry helpers
    # -------------------------------------------------------------------

    def estimate_distance(self, target_width_px: float) -> float | None:
        """
        Estimate distance to target using the similar-triangles formula:
            distance = (known_width * focal_length) / apparent_width_in_pixels
        Returns None if width is invalid.
        """
        if target_width_px <= 0:
            return None
        return (self.KNOWN_TARGET_WIDTH_M * self.FOCAL_LENGTH_PX) / target_width_px

    def estimate_angle(self, target_center_x_px: float) -> float:
        """
        Estimate horizontal angle from image centre to target [rad].
        Positive angle → target is to the right of centre.
        """
        offset_px = target_center_x_px - (self.IMAGE_WIDTH_PX / 2.0)
        return np.arctan2(offset_px, self.FOCAL_LENGTH_PX)

    # -------------------------------------------------------------------
    # Command helpers
    # -------------------------------------------------------------------

    def stop_robot(self):
        """Publish a zero Twist to halt the robot."""
        self.cmd_pub.publish(Twist())

    def publish_command(self, distance: float, angle: float):
        """
        Convert distance and angle errors into a Twist command and publish.

        linear.x  → forward/backward speed [m/s]
        angular.z → yaw rate [rad/s]  (positive = counter-clockwise)
        """
        distance_error = distance - self.TARGET_DISTANCE_M

        linear_x  =  self.K_LINEAR  * distance_error
        angular_z = -self.K_ANGULAR * angle          # negative: steer toward target

        # Clamp to safe limits
        linear_x  = float(np.clip(linear_x,  -self.MAX_LINEAR_SPEED,  self.MAX_LINEAR_SPEED))
        angular_z = float(np.clip(angular_z, -self.MAX_ANGULAR_SPEED, self.MAX_ANGULAR_SPEED))

        cmd = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    # -------------------------------------------------------------------
    # Main callback
    # -------------------------------------------------------------------

    def image_callback(self, msg: Image):
        # Convert ROS Image → OpenCV BGR frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # --- Detection (colour blob) ------------------------------------
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.HSV_LOWER, self.HSV_UPPER)

        # Optional morphological clean-up to remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter by minimum area
        valid = [c for c in contours if cv2.contourArea(c) >= self.MIN_CONTOUR_AREA]

        if not valid:
            self.get_logger().warn('No target detected — stopping robot.')
            self.stop_robot()
            return

        # Use the largest valid contour as the target
        target_contour = max(valid, key=cv2.contourArea)
        x, y, w, h     = cv2.boundingRect(target_contour)
        target_center_x = x + w / 2.0

        # --- Calculations -----------------------------------------------
        distance = self.estimate_distance(w)
        angle    = self.estimate_angle(target_center_x)

        if distance is None:
            self.stop_robot()
            return

        self.get_logger().info(
            f'Distance: {distance:.3f} m | '
            f'Angle: {np.degrees(angle):+.1f} deg | '
            f'Target px width: {w}'
        )

        # --- Publish command --------------------------------------------
        self.publish_command(distance, angle)


# -----------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = VisionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down vision control node.')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
