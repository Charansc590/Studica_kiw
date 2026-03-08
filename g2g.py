#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time


def normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class GoToGoal(Node):
    def __init__(self, xg=0.0, yg=0.0, thg_deg=0.0):
        super().__init__('go_to_goal_hack')

        # ROS2 publishers/subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # ---------------- TARGET ----------------
        self.xg = xg
        self.yg = yg
        self.thg = math.radians(thg_deg)

        # ---------------- GAINS (PD CONTROL) ----------------
        self.kp = 2.0
        self.kd = 0.01
        self.kp_th = 2.50
        self.kd_th = 0.1

        # ---------------- LIMITS ----------------
        self.vmax = 0.80
        self.wmax = 1.0

        # ---------------- TOLERANCES ----------------
        self.pos_tol = 0.0035
        self.ang_tol = 0.0055

        # ---------------- MIN SPEED (ANTI-DEADZONE) ----------------
        self.min_speed = 0.4
        self.min_ang_speed = 0.40
        self.slow_dist = 0.0005

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.start_x = None
        self.start_y = None
        self.min_travel = 0.03

        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_eth = 0.0

        self.done = False
        self.prev_time = time.time()

        # -------- STUCK DETECTION (RECOVERY) --------
        self.last_movement_pos = None
        self.last_movement_time = None
        self.movement_threshold = 0.01  # 1cm to consider as movement
        self.stuck_timeout = 10.0  # seconds

        # EDIT 1: Removed self.create_timer() — the old timer ran loop() in the
        # background automatically. This conflicted with sequential use in run.py
        # because spin() would never return. Now run() manually calls loop() + spin_once().

        self.get_logger().info(
            f"⚡ GoToGoalHack: target=({xg:.3f}, {yg:.3f}, {thg_deg:.1f}°)"
        )

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.th = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y

    def run(self):
        """EDIT 2: Added run() method — blocks until goal reached, then returns.
        Uses spin_once so run.py keeps control and can move to next task."""
        while not self.done and rclpy.ok():
            self.loop()
            rclpy.spin_once(self, timeout_sec=0.05)
        self.cmd_pub.publish(Twist())
        print(f"✅ GoToGoalHack done: ({self.xg:.3f}, {self.yg:.3f}, {math.degrees(self.thg):.1f}°)")

    def loop(self):
        if self.done:
            # Publish zero velocity to ensure the robot stops
            self.cmd_pub.publish(Twist())
            return

        if self.start_x is None:
            return

        now = time.time()
        dt = now - self.prev_time
        if dt <= 0.0:
            return

        dx = self.xg - self.x
        dy = self.yg - self.y
        eth = normalize(self.thg - self.th)

        ex =  math.cos(self.th) * dx + math.sin(self.th) * dy
        ey = -math.sin(self.th) * dx + math.cos(self.th) * dy
        dist = math.hypot(ex, ey)
        travel = math.hypot(self.x - self.start_x, self.y - self.start_y)

        # -------- STUCK DETECTION --------
        if self.last_movement_pos is None:
            self.last_movement_pos = (self.x, self.y)
            self.last_movement_time = now
        else:
            movement_dist = math.hypot(
                self.x - self.last_movement_pos[0],
                self.y - self.last_movement_pos[1]
            )
            if movement_dist > self.movement_threshold:
                # Robot moved, reset timer
                self.last_movement_pos = (self.x, self.y)
                self.last_movement_time = now
            elif (now - self.last_movement_time) > self.stuck_timeout:
                # No meaningful movement for 10 seconds
                self.cmd_pub.publish(Twist())
                self.get_logger().warn(
                    f"⚠️  STUCK RECOVERY: Robot stuck at ({self.x:.3f}, {self.y:.3f}) "
                    f"for {self.stuck_timeout}s. Cancelling goal."
                )
                self.done = True
                return

        if (
            abs(ex)  < self.pos_tol and
            abs(ey)  < self.pos_tol and
            abs(eth) < self.ang_tol and
            travel   > self.min_travel
        ):
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f"🎯 Goal reached at x={self.x:.3f}, y={self.y:.3f}, "
                f"th={math.degrees(self.th):.2f}°"
            )
            self.done = True
            # EDIT 3: Removed rclpy.shutdown() — if this ran here, ROS2 would die
            # after Task 1 and crash all remaining tasks. run.py calls shutdown at the end.
            return

        if abs(ex) > self.pos_tol:
            dex = (ex - self.prev_ex) / dt
            vx = self.kp * ex + self.kd * dex
            if dist > self.slow_dist and abs(vx) < self.min_speed:
                vx = math.copysign(self.min_speed, vx)
        else:
            vx = 0.0
        self.prev_ex = ex

        # Add small deadband to suppress noise in y
        if abs(ey) < 0.005:  # 5mm deadband
            ey = 0.0

        if abs(ey) > self.pos_tol:
            vy = 0.8 * self.kp * ey  # Reduced gain for y, no derivative
            if dist > self.slow_dist and abs(vy) < self.min_speed:
                vy = math.copysign(self.min_speed, vy)
        else:
            vy = 0.0
        self.prev_ey = ey

        if abs(eth) > self.ang_tol:
            deth = (eth - self.prev_eth) / dt
            wz = self.kp_th * eth + self.kd_th * deth
            if abs(wz) < self.min_ang_speed:
                wz = math.copysign(self.min_ang_speed, wz)
        else:
            wz = 0.0
        self.prev_eth = eth

        vx = max(min(vx, self.vmax), -self.vmax)
        vy = max(min(vy, self.vmax), -self.vmax)
        wz = max(min(wz, self.wmax), -self.wmax)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = -vy
        cmd.angular.z = -wz
        self.cmd_pub.publish(cmd)
        self.prev_time = now
