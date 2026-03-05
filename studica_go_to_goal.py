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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # -------- TARGET --------
        self.xg = xg
        self.yg = yg
        self.thg = math.radians(thg_deg)

        # -------- GAINS --------
        self.kp = 1.3  
        self.kd = 0.01
        self.kp_th = 2.0
        self.kd_th = 0.1

        # -------- LIMITS --------
        self.vmax = 0.7
        self.wmax = 1.0

        # -------- TOLERANCES --------
        self.pos_tol = 0.05
        self.ang_tol = 0.05

        # -------- MIN SPEED --------
        self.min_speed = 0.25 
        self.min_ang_speed = 0.40

        # -------- SLOW ZONE --------
        self.slow_dist = 0.10
        self.align_dist = 0.05

        # -------- STATE --------
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.start_x = None
        self.start_y = None
        self.min_travel = 0.03

        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_eth = 0.0

        self.prev_vx = 0.0
        self.prev_vy = 0.0
        self.prev_wz = 0.0

        self.acc_lin = 0.2
        self.acc_ang = 0.2

        self.done = False
        self.prev_time = time.time()
        
        # Safety timeout (seconds)
        self.goal_timeout = 15.0
        self.goal_start_time = time.time()

        self.get_logger().info(
            f"⚡ GoToGoalHack target=({xg:.3f},{yg:.3f},{thg_deg:.1f}°)"
        )


    def odom_cb(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.th = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y


    def run(self):

        # -------- WAIT FOR ODOMETRY SYNC --------
        odom_ready = False
        sync_attempts = 0
        max_attempts = 50

        while not odom_ready and rclpy.ok() and sync_attempts < max_attempts:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.start_x is not None:
                odom_ready = True
            sync_attempts += 1

        if not odom_ready:
            self.get_logger().error("❌ Odometry sync failed!")
            return

        self.get_logger().info(
            f"📍 Odom synced at ({self.x:.3f},{self.y:.3f},{math.degrees(self.th):.2f}°)"
        )

        # -------- INITIALIZE TIMING --------
        self.prev_time = time.time()

        # -------- CONTROL LOOP --------
        while not self.done and rclpy.ok():

            self.loop()
            rclpy.spin_once(self, timeout_sec=0.05)

        self.cmd_pub.publish(Twist())
        time.sleep(0.1)  # Ensure stop command is published

        print(
            f"✅ Done ({self.xg:.3f},{self.yg:.3f},{math.degrees(self.thg):.1f}°)"
        )


    def loop(self):

        if self.done or self.start_x is None:
            return

        # -------- TIMEOUT SAFETY --------
        elapsed_time = time.time() - self.goal_start_time
        if elapsed_time > self.goal_timeout:
            self.get_logger().warning(
                f"⏱️  Goal timeout ({elapsed_time:.1f}s > {self.goal_timeout}s). Stopping."
            )
            self.cmd_pub.publish(Twist())
            self.done = True
            return

        now = time.time()
        dt = now - self.prev_time
        if dt <= 0:
            return

        dx = self.xg - self.x
        dy = self.yg - self.y
        eth = normalize(self.thg - self.th)

        ex = math.cos(self.th)*dx + math.sin(self.th)*dy
        ey = -math.sin(self.th)*dx + math.cos(self.th)*dy

        dist = math.hypot(ex,ey)
        travel = math.hypot(self.x-self.start_x,self.y-self.start_y)
        
        # Effective minimum travel: lower of min_travel or half the initial distance
        initial_dist = math.hypot(self.xg-self.start_x, self.yg-self.start_y)
        min_travel_threshold = min(self.min_travel, initial_dist * 0.5) if initial_dist > 0.01 else 0.01

        # -------- FINAL STOP --------
        goal_reached = (
            abs(ex) < self.pos_tol and
            abs(ey) < self.pos_tol and
            abs(eth) < self.ang_tol and
            travel > min_travel_threshold
        )
        
        if goal_reached:

            self.cmd_pub.publish(Twist())

            self.get_logger().info(
                f"🎯 Goal reached x={self.x:.3f} y={self.y:.3f} th={math.degrees(self.th):.2f}"
            )

            self.done = True
            return


        # -------- ALIGNMENT MODE --------
        if dist < self.align_dist:
            ex = 0
            ey = 0


        # -------- X CONTROL --------
        if abs(ex) > self.pos_tol:

            dex = (ex-self.prev_ex)/dt
            vx = self.kp*ex + self.kd*dex

            if abs(vx) < self.min_speed:
                vx = math.copysign(self.min_speed,vx)

        else:
            vx = 0

        self.prev_ex = ex


        # -------- Y CONTROL --------
        if abs(ey) > self.pos_tol:

            dey = (ey-self.prev_ey)/dt
            vy = self.kp*ey + self.kd*dey

            if abs(vy) < self.min_speed:
                vy = math.copysign(self.min_speed,vy)

        else:
            vy = 0

        self.prev_ey = ey


        # -------- ROTATION --------
        if abs(eth) > self.ang_tol:

            deth = (eth-self.prev_eth)/dt
            wz = self.kp_th*eth + self.kd_th*deth

            if abs(wz) < self.min_ang_speed:
                wz = math.copysign(self.min_ang_speed,wz)

        else:
            wz = 0

        self.prev_eth = eth


        # -------- LIMITS --------
        vx = max(min(vx,self.vmax),-self.vmax)
        vy = max(min(vy,self.vmax),-self.vmax)
        wz = max(min(wz,self.wmax),-self.wmax)


        # -------- RAMP FILTER --------
        dvx = max(min(vx-self.prev_vx,self.acc_lin),-self.acc_lin)
        dvy = max(min(vy-self.prev_vy,self.acc_lin),-self.acc_lin)
        dwz = max(min(wz-self.prev_wz,self.acc_ang),-self.acc_ang)

        vx = self.prev_vx + dvx
        vy = self.prev_vy + dvy
        wz = self.prev_wz + dwz

        self.prev_vx = vx
        self.prev_vy = vy
        self.prev_wz = wz


        # -------- PUBLISH --------
        cmd = Twist()

        cmd.linear.x = vx
        cmd.linear.y = -vy
        cmd.angular.z = -wz

        self.cmd_pub.publish(cmd)

        self.prev_time = now





# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# import math
# import time


# def normalize(a):
#     while a > math.pi:
#         a -= 2 * math.pi
#     while a < -math.pi:
#         a += 2 * math.pi
#     return a


# class GoToGoal(Node):

#     def __init__(self, xg=0.0, yg=0.0, thg_deg=0.0):
#         super().__init__('go_to_goal')

#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

#         self.xg = xg
#         self.yg = yg
#         self.thg = math.radians(thg_deg)

#         # Gains
#         self.kp = 1.2
#         self.kd = 0.02
#         self.kp_th = 1.2
#         self.kd_th = 0.02

#         # Speed limits
#         self.vmax = 0.7
#         self.wmax = 0.8
#         self.min_lin = 0.2
#         self.min_ang = 0.4

#         # Tolerances
#         self.pos_tol = 0.02
#         self.ang_tol = 0.02

#         # Deadband
#         self.dead_lin = 0.03
#         self.dead_ang = 0.03

#         # State
#         self.x = None
#         self.y = None
#         self.th = None

#         self.prev_dx = 0.0
#         self.prev_dy = 0.0
#         self.prev_eth = 0.0

#         self.done = False
#         self.prev_time = None

#         # derivative filtering
#         self.alpha = 0.3
#         self.ddx_f = 0.0
#         self.ddy_f = 0.0
#         self.deth_f = 0.0

#         self.initial_dist = None

#         # ramp
#         self.prev_vx = 0.0
#         self.prev_vy = 0.0
#         self.prev_wz = 0.0

#         self.lin_acc = 0.05
#         self.ang_acc = 0.05


#     def odom_cb(self, msg):

#         self.x = msg.pose.pose.position.x
#         self.y = msg.pose.pose.position.y

#         q = msg.pose.pose.orientation

#         self.th = math.atan2(
#             2 * (q.w*q.z + q.x*q.y),
#             1 - 2 * (q.y*q.y + q.z*q.z)
#         )


#     def run(self):

#         print("[GoToGoal] Syncing odom...")

#         odom_count = 0

#         while odom_count < 5 and rclpy.ok():

#             rclpy.spin_once(self, timeout_sec=0.1)

#             if self.x is not None:
#                 odom_count += 1


#         print(f"[GoToGoal] Odom ready: ({self.x:.3f},{self.y:.3f},{math.degrees(self.th):.1f}°)")

#         dx = self.xg - self.x
#         dy = self.yg - self.y

#         self.initial_dist = math.hypot(dx, dy)

#         self.prev_time = time.time()

#         while not self.done and rclpy.ok():

#             rclpy.spin_once(self, timeout_sec=0.05)
#             self.loop()


#         self.cmd_pub.publish(Twist())
#         rclpy.spin_once(self, timeout_sec=0.02)
#         self.cmd_pub.publish(Twist())

#         print(f"[GoToGoal] Done → ({self.xg:.3f},{self.yg:.3f},{math.degrees(self.thg):.1f}°)")


#     def loop(self):

#         if self.done or self.x is None:
#             return


#         now = time.time()

#         dt = now - self.prev_time

#         if dt < 0.01:
#             return

#         self.prev_time = now


#         dx = self.xg - self.x
#         dy = self.yg - self.y
#         eth = normalize(self.thg - self.th)


#         # -------- lateral noise suppression --------
#         if abs(dy) < self.dead_lin and abs(dx) > 0.05:
#             dy = 0.0


#         # -------- zero error inside tolerance --------
#         if abs(dx) < self.pos_tol:
#             dx = 0.0

#         if abs(dy) < self.pos_tol:
#             dy = 0.0


#         dist = math.hypot(dx, dy)


#         if dist < self.pos_tol and abs(eth) < self.ang_tol:

#             self.cmd_pub.publish(Twist())
#             self.done = True
#             return


#         slow = min(dist / 0.15, 1.0)


#         # -------- PD world frame --------

#         raw_ddx = (dx - self.prev_dx) / dt
#         raw_ddy = (dy - self.prev_dy) / dt
#         raw_deth = (eth - self.prev_eth) / dt

#         self.ddx_f = self.alpha * raw_ddx + (1 - self.alpha) * self.ddx_f
#         self.ddy_f = self.alpha * raw_ddy + (1 - self.alpha) * self.ddy_f
#         self.deth_f = self.alpha * raw_deth + (1 - self.alpha) * self.deth_f


#         vx_world = (self.kp * dx + self.kd * self.ddx_f) * slow
#         vy_world = (self.kp * dy + self.kd * self.ddy_f) * slow

#         wz = self.kp_th * eth + self.kd_th * self.deth_f


#         self.prev_dx = dx
#         self.prev_dy = dy
#         self.prev_eth = eth


#         # -------- world → robot transform --------

#         cos_th = math.cos(self.th)
#         sin_th = math.sin(self.th)

#         vx = cos_th * vx_world + sin_th * vy_world
#         vy = -sin_th * vx_world + cos_th * vy_world


#         # -------- minimum speed --------

#         if 0 < abs(vx) < self.min_lin:
#             vx = math.copysign(self.min_lin, vx)

#         if 0 < abs(vy) < self.min_lin:
#             vy = math.copysign(self.min_lin, vy)

#         if 0 < abs(wz) < self.min_ang:
#             wz = math.copysign(self.min_ang, wz)


#         # -------- ramp --------

#         dvx = max(min(vx - self.prev_vx, self.lin_acc), -self.lin_acc)
#         dvy = max(min(vy - self.prev_vy, self.lin_acc), -self.lin_acc)
#         dwz = max(min(wz - self.prev_wz, self.ang_acc), -self.ang_acc)

#         vx = self.prev_vx + dvx
#         vy = self.prev_vy + dvy
#         wz = self.prev_wz + dwz

#         self.prev_vx = vx
#         self.prev_vy = vy
#         self.prev_wz = wz


#         vx = max(min(vx, self.vmax), -self.vmax)
#         vy = max(min(vy, self.vmax), -self.vmax)
#         wz = max(min(wz, self.wmax), -self.wmax)


#         cmd = Twist()

#         cmd.linear.x = vx
#         cmd.linear.y = -vy
#         cmd.angular.z = -wz

#         self.cmd_pub.publish(cmd)


#         print(
#             f"x={self.x:.3f} y={self.y:.3f} th={math.degrees(self.th):.2f}° | "
#             f"dx={dx:.3f} dy={dy:.3f} | "
#             f"vx={vx:.3f} vy={-vy:.3f} wz={-wz:.3f}"
#         )
