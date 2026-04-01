#!/usr/bin/env python3
import time
import serial
import serial.tools.list_ports
import serial.serialutil

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.SERIAL_PORT = '/dev/ttyACM0'
        self.BAUD_RATE = 115200   # MUST match the working test node / Arduino
        self.SEND_PERIOD = 0.2    # 5 Hz, same as working test node

        # -----------------------------
        # Topic state
        # -----------------------------
        self.detected = 0
        self.centering = '0'

        # Turning commands
        # positive omega = left
        # negative omega = right
        self.OMEGA_LEFT = 0.8
        self.OMEGA_RIGHT = -0.8

        self.ser = None
        self._connect_serial()

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.detected_sub = self.create_subscription(
            Int32,
            '/yellow_detector/detected',
            self.detected_callback,
            10
        )

        self.centering_sub = self.create_subscription(
            String,
            '/yellow_detector/centering',
            self.centering_callback,
            10
        )

        # -----------------------------
        # Timer
        # -----------------------------
        self.timer = self.create_timer(self.SEND_PERIOD, self.timer_callback)

        self.get_logger().info('Arduino bridge node started.')
        self.get_logger().info('Subscribed to /yellow_detector/detected and /yellow_detector/centering')

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(
                self.SERIAL_PORT,
                self.BAUD_RATE,
                timeout=1
            )

            self.get_logger().info('Waiting for Arduino to boot...')
            time.sleep(3)

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.get_logger().info('Waiting for Arduino ready signal...')
            deadline = time.time() + 10.0

            while time.time() < deadline:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.get_logger().info(f'Arduino says: {line}')
                if 'ready' in line.lower():
                    self.get_logger().info('Arduino is ready. Starting commands.')
                    break
            else:
                self.get_logger().warn('Timed out waiting for Arduino ready signal. Proceeding anyway.')

        except serial.SerialException:
            available = [p.device for p in serial.tools.list_ports.comports()]
            self.get_logger().error(
                f'Could not open {self.SERIAL_PORT}. Available ports: {available}'
            )
            self.ser = None

    def detected_callback(self, msg: Int32):
        self.detected = 1 if msg.data == 1 else 0

    def centering_callback(self, msg: String):
        value = msg.data.strip().lower()

        if value in ['left', 'right', '0']:
            self.centering = value
        else:
            self.centering = '0'
            self.get_logger().warn(f'Unexpected centering value "{msg.data}", using 0')

    def compute_command(self):
        # Forward speed
        if self.detected == 1:
            v_cmd = 0.5000
        else:
            v_cmd = 0.0000

        # Turn rate
        if self.centering == 'left':
            omega_cmd = self.OMEGA_LEFT
        elif self.centering == 'right':
            omega_cmd = self.OMEGA_RIGHT
        else:
            omega_cmd = 0.0000

        return v_cmd, omega_cmd

    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not open.')
            return

        v_cmd, w_cmd = self.compute_command()
        cmd = f"v:{v_cmd:.4f},w:{w_cmd:.4f}\n"

        try:
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info(
                f'Sent: {cmd.strip()}   '
                f'(detected={self.detected}, centering={self.centering})'
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"v:0.0000,w:0.0000\n")
                self.get_logger().info('Sent stop command before shutdown.')
            except serial.SerialException:
                pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down arduino bridge node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
