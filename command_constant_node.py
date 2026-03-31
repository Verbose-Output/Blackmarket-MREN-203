#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
import time


class ConstantCommandNode(Node):
    def __init__(self):
        super().__init__('constant_command')

        self.SERIAL_PORT = '/dev/ttyACM0'
        self.BAUD_RATE = 9600
        self.ser = None

        self.v_cmd = 0.5
        self.w_cmd = 0.0

        self._connect_serial()

        # Send the command repeatedly
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info('Constant command node started.')
        self.get_logger().info(f'Sending v={self.v_cmd}, w={self.w_cmd}')

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(
                self.SERIAL_PORT,
                self.BAUD_RATE,
                timeout=1
            )
            time.sleep(2)
            self.get_logger().info(f'Connected to Arduino on {self.SERIAL_PORT}')
        except serial.SerialException:
            available = [p.device for p in serial.tools.list_ports.comports()]
            self.get_logger().error(
                f'Could not open {self.SERIAL_PORT}. Available ports: {available}'
            )
            self.ser = None

    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial port not open.')
            return

        cmd = f"v:{self.v_cmd:.4f},w:{self.w_cmd:.4f}\n"

        try:
            self.ser.write(cmd.encode('utf-8'))
            self.get_logger().info(f'Sent: {cmd.strip()}')
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
    node = ConstantCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down constant command node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
