#!/usr/bin/env python3
"""
arduino_bridge_node.py
----------------------
ROS 2 Humble node that:
  1. Subscribes to /cmd_vel (Twist messages from vision_control_node)
  2. Forwards velocity commands to the Arduino over USB serial (TX only)

The Arduino handles all wheel-level PI control internally.
No data needs to be read back from the Arduino — the camera provides feedback.

Serial command format sent to Arduino:
    "v:{linear_x:.4f},w:{angular_z:.4f}\n"

    v  → desired vehicle speed   [m/s]
    w  → desired vehicle yaw rate [rad/s]

These map directly to v_desired and omega_D in the Arduino PI controller.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import time


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # ---------------------------------------------------------------
        # Serial configuration
        # ---------------------------------------------------------------
        self.SERIAL_PORT = '/dev/ttyACM0'   # change to ttyUSB0 if needed
        self.BAUD_RATE   = 9600             # must match Arduino Serial.begin()
        self.ser         = None

        self._connect_serial()

        # ---------------------------------------------------------------
        # ROS interface — subscribe only, no publisher needed
        # ---------------------------------------------------------------
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.get_logger().info('Arduino bridge node started (TX only).')
        self.get_logger().info(f'Serial port: {self.SERIAL_PORT} @ {self.BAUD_RATE} baud')

    # -------------------------------------------------------------------
    # Serial helpers
    # -------------------------------------------------------------------

    def _connect_serial(self):
        """
        Attempt to open the serial port.
        Lists available ports if the expected one is not found.
        """
        try:
            self.ser = serial.Serial(
                self.SERIAL_PORT,
                self.BAUD_RATE,
                timeout=1
            )
            # Wait for Arduino to finish resetting after USB connection
            time.sleep(2)
            self.get_logger().info(f'Connected to Arduino on {self.SERIAL_PORT}')
        except serial.SerialException:
            # List available ports to help the user find the correct one
            available = [p.device for p in serial.tools.list_ports.comports()]
            self.get_logger().error(
                f'Could not open {self.SERIAL_PORT}. '
                f'Available ports: {available}'
            )
            self.ser = None

    def _send(self, v: float, omega: float):
        """
        Format and send a velocity command to the Arduino.
        Format: "v:{v:.4f},w:{omega:.4f}\n"
        """
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not open — skipping command.')
            return

        cmd = f"v:{v:.4f},w:{omega:.4f}\n"
        try:
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().debug(f'Sent: {cmd.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def stop_robot(self):
        """Send a zero-velocity command to halt the Arduino."""
        self._send(0.0, 0.0)

    # -------------------------------------------------------------------
    # ROS callback
    # -------------------------------------------------------------------

    def cmd_callback(self, msg: Twist):
        """
        Receive a Twist from /cmd_vel and forward to Arduino.

        msg.linear.x  → v       (forward speed [m/s])
        msg.angular.z → omega_D (yaw rate      [rad/s])
        """
        v     = msg.linear.x
        omega = msg.angular.z
        self._send(v, omega)

    # -------------------------------------------------------------------
    # Cleanup
    # -------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Stopping robot and closing serial port.')
        self.stop_robot()
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


# -----------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Arduino bridge node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
