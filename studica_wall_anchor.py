#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class KiwiAnchorPD(Node):

    def __init__(self, forward_dist=15.0, side_dist=-15.0):
        super().__init__('kiwi_anchor_pd')

        # ================= PARAMETERS =================
        self.target_distance = forward_dist
        self.target_distance_side = abs(side_dist)
        self.side = 'left' if side_dist <= 0 else 'right'

        # ---- PD Gains ----
        self.Kp_d = 0.2
        self.Kd_d = 0.002
        self.Kp_l = self.Kp_d
        self.Kd_l = self.Kd_d
        self.Kp_a = 0.2
        self.Kd_a = 0.02

        # ---- Deadbands (cm) ----
        self.distance_deadband = 0.5
        self.left_deadband = 0.5
        self.align_deadband = 0.5

        # ---- Tolerance ----
        self.distance_tolerance = 0.1
        self.left_tolerance = 0.1
        self.align_tolerance = 0.1

        # ---- Output Limits ----
        self.max_linear = 0.27
        self.max_angular = 0.5

        # ---- Timing ----
        self.dt = 1.0 / 50.0

        # ---- Median Filter Buffers ----
        self.filter_window = 15
        self.fl_buffer = []
        self.fr_buffer = []
        self.side_buffer = []

        # ---- Success Detection ----
        self.anchor_success_frames = 0
        self.anchor_success_threshold = 25

        # ---- Explore velocity ----
        self.explore_velocity = 0.05

        # ================= VARIABLES =================
        self.fl_dis = 0.0
        self.fr_dis = 0.0
        self.side_dis = 0.0

        self.prev_ed = 0.0
        self.prev_ea = 0.0
        self.prev_el = 0.0

        self.valid_data = False
        self.anchored = False          # <-- SUCCESS FLAG: run() loop exits when True

        # ---- Timeout for stuck sensor data ----
        self.timeout_duration = 5.0  # seconds
        self.timeout_counter = 0.0
        self.prev_sensor_data = [-1, -1, -1, -1]

        # ================= ROS =================
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32MultiArray, '/dis_data', self.dis_callback, 10)
        self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"Wall Anchor Started | Front: {self.target_distance}cm | "
            f"{self.side.upper()}: {self.target_distance_side}cm"
        )

    # =====================================================
    # RUN — spins until anchored, then returns to run.py
    # =====================================================
    def run(self):
        while rclpy.ok() and not self.anchored:   # <-- exits as soon as anchored=True
            rclpy.spin_once(self, timeout_sec=0.02)
        self.destroy_node()
        return self.anchored   # returns True on success

    # =====================================================
    # MEDIAN FILTER
    # =====================================================
    def apply_median_filter(self, new_value, buffer):
        if new_value != -1:
            buffer.append(new_value)
        if len(buffer) > self.filter_window:
            buffer.pop(0)
        valid_values = sorted([v for v in buffer if v != -1])
        if not valid_values:
            return -1
        mid = len(valid_values) // 2
        if len(valid_values) % 2 == 0:
            return (valid_values[mid - 1] + valid_values[mid]) / 2.0
        return valid_values[mid]

    # =====================================================
    # SENSOR CALLBACK
    # =====================================================
    def dis_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        new_fl   = msg.data[1]
        new_fr   = msg.data[2]
        new_side = msg.data[0] if self.side == 'left' else msg.data[3]

        self.fl_dis   = self.apply_median_filter(new_fl,   self.fl_buffer)
        self.fr_dis   = self.apply_median_filter(new_fr,   self.fr_buffer)
        self.side_dis = self.apply_median_filter(new_side, self.side_buffer)

        if self.fl_dis != -1 or self.fr_dis != -1 or self.side_dis != -1:
            self.valid_data = True

        # Check if sensor data has changed significantly (>= 0.5 to avoid noise)
        current_sensor_data = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]
        data_changed = False
        
        for i in range(4):
            if self.prev_sensor_data[i] != -1 and current_sensor_data[i] != -1:
                if abs(current_sensor_data[i] - self.prev_sensor_data[i]) >= 0.5:
                    data_changed = True
                    self.get_logger().debug(f"Significant change at index {i}: {self.prev_sensor_data[i]} -> {current_sensor_data[i]}")
                    break
        
        if data_changed:
            self.timeout_counter = 0.0
        
        self.prev_sensor_data = current_sensor_data

    # =====================================================
    # CONTROL LOOP
    # =====================================================
    def control_loop(self):
        if not self.valid_data:
            return

        # Increment timeout counter at each control loop cycle (runs at 50Hz)
        self.timeout_counter += self.dt

        # Check for sensor data timeout
        if self.timeout_counter >= self.timeout_duration:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().error(
                f"✗ TIMEOUT: Sensor data has not changed significantly for {self.timeout_duration}s. Exiting."
            )
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

        twist = Twist()

        # --- Front wall distance ---
        if self.fl_dis == -1 or self.fr_dis == -1:
            vx = self.explore_velocity
            ed = 999
        else:
            d_avg = (self.fl_dis + self.fr_dis) / 2.0
            ed = self.target_distance - d_avg
            if abs(ed) < self.distance_deadband:
                ed = 0.0
            vx = self.Kp_d * ed + self.Kd_d * (ed - self.prev_ed) / self.dt
            self.prev_ed = ed

        # --- Side wall distance ---
        if self.side_dis == -1:
            vy = self.explore_velocity
            el = 999
        else:
            el = self.target_distance_side - self.side_dis
            if abs(el) < self.left_deadband:
                el = 0.0
            vy = self.Kp_l * el + self.Kd_l * (el - self.prev_el) / self.dt
            self.prev_el = el

        if self.side == 'right':
            vy = -vy

        # --- Alignment ---
        if self.fl_dis == -1 or self.fr_dis == -1:
            ea = 0.0
            wz = 0.0
        else:
            ea = self.fl_dis - self.fr_dis
            if abs(ea) < self.align_deadband:
                ea = 0.0
            wz = self.Kp_a * ea + self.Kd_a * (ea - self.prev_ea) / self.dt
            self.prev_ea = ea

        # --- Clamp ---
        vx = max(min(vx, self.max_linear), -self.max_linear)
        vy = max(min(vy, self.max_linear), -self.max_linear)
        wz = max(min(wz, self.max_angular), -self.max_angular)

        # --- Check success ---
        if (self.fl_dis != -1 and self.fr_dis != -1 and self.side_dis != -1 and
                abs(ed) < self.distance_tolerance and
                abs(el) < self.left_tolerance and
                abs(ea) < self.align_tolerance):

            self.anchor_success_frames += 1

            if self.anchor_success_frames >= self.anchor_success_threshold:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self.get_logger().info(
                    f"✓ ANCHOR SUCCESS | "
                    f"Front: {(self.fl_dis + self.fr_dis) / 2.0:.2f}cm | "
                    f"{self.side.upper()}: {self.side_dis:.2f}cm | "
                    f"Align: {ea:.2f}cm"
                )
                self.anchored = True   # <-- triggers run() loop to exit
                return
        else:
            self.anchor_success_frames = 0

        twist.linear.x = -vx
        twist.linear.y = -vy
        twist.angular.z = - wz
        self.cmd_pub.publish(twist)

        fl_str   = "SCAN" if self.fl_dis   == -1 else f"{self.fl_dis:.2f}"
        fr_str   = "SCAN" if self.fr_dis   == -1 else f"{self.fr_dis:.2f}"
        side_str = "SCAN" if self.side_dis == -1 else f"{self.side_dis:.2f}"
        self.get_logger().info(
            f"FL={fl_str} FR={fr_str} {self.side.upper()}={side_str} | ")
        #     f"Err_d={ed:.2f} Err_l={el:.2f} Err_a={ea:.2f} | "
        #     f"Vx={vx:.3f} Vy={vy:.3f} Wz={wz:.3f}"
        # )


def main():
    rclpy.init()
    node = KiwiAnchorPD()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
