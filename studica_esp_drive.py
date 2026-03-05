#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_serial')

        # -------- SERIAL CONFIG --------
        self.port = '/dev/ttyACM1'   # change if needed
        self.baud = 115200

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            exit(1)

        # -------- SUBSCRIBER --------
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmdvel_callback,
            10
        )

        # -------- WATCHDOG TIMER --------
        self.last_cmd_time = self.get_clock().now()
        # self.watchdog_timeout = 0.5  # seconds
        # self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

    def cmdvel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        vx =  msg.linear.x
        vy =  msg.linear.y
        wz =  msg.angular.z

        data = f"{vx:.3f},{vy:.3f},{wz:.3f}\n"
        print(f"Publishing to ESP32: {data.strip()}")
        self.ser.write(data.encode())

        # Debug (optional)
        self.get_logger().debug(f"Sent: {data.strip()}")

def main():
    rclpy.init()
    node = CmdVelToSerial()
    rclpy.spin(node)

    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
