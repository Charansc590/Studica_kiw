#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import tf_transformations
import tf2_ros
from gpiozero import RotaryEncoder
import serial
import struct
import math
import time

# --- Utility ---
def normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

# --- Odometry + IMU Node ---
class EncoderIMUFused(Node):
    def __init__(self):
        super().__init__('encoder_imu_fused')
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.ser.setDTR(False)
            time.sleep(0.2)
            self.ser.setDTR(True)
            time.sleep(2.0)
        except Exception as e:
            self.ser = None
        self.packet_size = 48
        self.header_size = 2
        self.CPR = 390
        self.WHEEL_DIAM = 0.056
        self.CIRC = math.pi * self.WHEEL_DIAM
        self.R = 0.27
        self.THETA_FL = math.radians(30)
        self.THETA_FR = math.radians(150)
        self.THETA_B  = math.radians(270)
        self.enc_fl = RotaryEncoder(5, 25, max_steps=0)
        self.enc_fr = RotaryEncoder(23, 24, max_steps=0)
        self.enc_b  = RotaryEncoder(17, 27, max_steps=0)
        self.prev_steps = {'fl': 0, 'fr': 0, 'b': 0}
        self.prev_time = time.time()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.dis_pub  = self.create_publisher(Float32MultiArray, '/dis_data', 10)
        self.imu_pub  = self.create_publisher(Imu, '/imu', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.update)

    def steps_to_velocity(self, dsteps, dt):
        return (dsteps / self.CPR) * self.CIRC / dt

    def read_serial_packet(self):
        if self.ser is None:
            return None
        try:
            while self.ser.in_waiting >= (self.header_size + self.packet_size):
                if self.ser.read(1) == b'\xAA':
                    if self.ser.read(1) == b'\x55':
                        data = self.ser.read(self.packet_size)
                        if len(data) == self.packet_size:
                            values = struct.unpack('12f', data)
                            return values
        except Exception:
            pass
        return None

    def update(self):
        now = time.time()
        dt = now - self.prev_time
        if dt <= 0:
            return
        serial_data = self.read_serial_packet()
        if serial_data is None:
            return
        yaw_unused = serial_data[0]
        wz_imu     = serial_data[1]
        ax, ay, az = serial_data[2], serial_data[3], serial_data[4]
        gx, gy, gz = serial_data[5], serial_data[6], serial_data[7]
        ir1, ir2, us1, us2 = serial_data[8], serial_data[9], serial_data[10], serial_data[11]
        self.th -= wz_imu * dt
        if self.th > math.pi:
            self.th -= 2 * math.pi
        if self.th < -math.pi:
            self.th += 2 * math.pi
        steps = {
            'fl': self.enc_fl.steps,
            'fr': self.enc_fr.steps,
            'b':  self.enc_b.steps
        }
        v = {}
        for k in steps:
            ds = steps[k] - self.prev_steps[k]
            v[k] = self.steps_to_velocity(ds, dt)
            self.prev_steps[k] = steps[k]
        w1 =  v['fl']
        w2 =  v['fr']
        w3 =  v['b']
        vx = (2.0/3.0) * (
            math.cos(self.THETA_FL) * w1 +
            math.cos(self.THETA_FR) * w2 +
            math.cos(self.THETA_B ) * w3
        )
        vy = (2.0/3.0) * (
            math.sin(self.THETA_FL) * w1 +
            math.sin(self.THETA_FR) * w2 +
            math.sin(self.THETA_B ) * w3
        )
        dx = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        dy = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        self.x += dx
        self.y += dy
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        q_tf = tf_transformations.quaternion_from_euler(0, 0, -self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz_imu
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = -self.y
        t.transform.rotation.x = q_tf[0]
        t.transform.rotation.y = q_tf[1]
        t.transform.rotation.z = q_tf[2]
        t.transform.rotation.w = q_tf[3]
        self.tf_broadcaster.sendTransform(t)
        imu_msg = Imu()
        imu_msg.header = odom.header
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        self.imu_pub.publish(imu_msg)
        dis_msg = Float32MultiArray()
        dis_msg.data = [ir1, ir2, us1, us2]
        self.dis_pub.publish(dis_msg)
        self.prev_time = now

# --- GoToGoal Node ---
class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal_hack')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        # Subscribe to waypoint topic (geometry_msgs/Pose)
        from std_msgs.msg import String
        self.create_subscription(
            String,
            '/kiwi/waypoints',
            self.wp_cb,
            10
        )
        self.xg = None
        self.yg = None
        self.thg = None
        self.kp = 1.3
        self.kd = 0.01
        self.kp_th = 2.0
        self.kd_th = 0.1
        self.vmax = 0.7
        self.wmax = 1.0
        self.pos_tol = 0.05
        self.ang_tol = 0.05
        self.min_speed = 0.25
        self.min_ang_speed = 0.40
        self.slow_dist = 0.05
        self.align_dist = 0.05
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
        self.done = True  # Start as done
        self.prev_time = time.time()
        self.goal_timeout = 15.0
        self.goal_start_time = time.time()
        # Create timer for goal control loop (50Hz / 0.02s)
        self.goal_timer = self.create_timer(0.02, self._goal_update)

    def wp_cb(self, msg):
        # Expect String: "x,y,theta_deg" (e.g. "1.0,0.0,90")
        try:
            parts = msg.data.strip().split(',')
            self.xg = float(parts[0])
            self.yg = float(parts[1])
            self.thg = math.radians(float(parts[2]))
            self.done = False
            self.start_x = None
            self.start_y = None
            self.goal_start_time = time.time()
        except Exception as e:
            pass

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

    def _goal_update(self):
        if self.done or self.start_x is None:
            return
        elapsed_time = time.time() - self.goal_start_time
        if elapsed_time > self.goal_timeout:
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
        initial_dist = math.hypot(self.xg-self.start_x, self.yg-self.start_y)
        min_travel_threshold = min(self.min_travel, initial_dist * 0.5) if initial_dist > 0.01 else 0.01
        goal_reached = (
            abs(ex) < self.pos_tol and
            abs(ey) < self.pos_tol and
            abs(eth) < self.ang_tol and
            travel > min_travel_threshold
        )
        if goal_reached:
            self.cmd_pub.publish(Twist())
            self.done = True
            return
        if dist < self.align_dist:
            ex = 0
            ey = 0
        if abs(ex) > self.pos_tol:
            dex = (ex-self.prev_ex)/dt
            vx = self.kp*ex + self.kd*dex
            if abs(vx) < self.min_speed:
                vx = math.copysign(self.min_speed,vx)
        else:
            vx = 0
        self.prev_ex = ex
        if abs(ey) > self.pos_tol:
            dey = (ey-self.prev_ey)/dt
            vy = self.kp*ey + self.kd*dey
            if abs(vy) < self.min_speed:
                vy = math.copysign(self.min_speed,vy)
        else:
            vy = 0
        self.prev_ey = ey
        if abs(eth) > self.ang_tol:
            deth = (eth-self.prev_eth)/dt
            wz = self.kp_th*eth + self.kd_th*deth
            if abs(wz) < self.min_ang_speed:
                wz = math.copysign(self.min_ang_speed,wz)
        else:
            wz = 0
        self.prev_eth = eth
        vx = max(min(vx,self.vmax),-self.vmax)
        vy = max(min(vy,self.vmax),-self.vmax)
        wz = max(min(wz,self.wmax),-self.wmax)
        dvx = max(min(vx-self.prev_vx,self.acc_lin),-self.acc_lin)
        dvy = max(min(vy-self.prev_vy,self.acc_lin),-self.acc_lin)
        dwz = max(min(wz-self.prev_wz,self.acc_ang),-self.acc_ang)
        vx = self.prev_vx + dvx
        vy = self.prev_vy + dvy
        wz = self.prev_wz + dwz
        self.prev_vx = vx
        self.prev_vy = vy
        self.prev_wz = wz
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = -vy
        cmd.angular.z = -wz
        self.cmd_pub.publish(cmd)
        self.prev_time = now

# --- Serial Velocity Node ---
class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_serial')
        self.port = '/dev/ttyACM1'
        self.baud = 115200
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)
        except Exception as e:
            exit(1)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmdvel_callback,
            10
        )
        self.last_cmd_time = self.get_clock().now()

    def cmdvel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        vx =  msg.linear.x
        vy =  msg.linear.y
        wz =  msg.angular.z
        data = f"{vx:.3f},{vy:.3f},{wz:.3f}\n"
        self.ser.write(data.encode())

# --- Main ---
def main():
    rclpy.init()
    odom_node = EncoderIMUFused()
    serial_node = CmdVelToSerial()
    goal_node = GoToGoal()  # No default goal
    executor = MultiThreadedExecutor()
    executor.add_node(odom_node)
    executor.add_node(serial_node)
    executor.add_node(goal_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            odom_node.destroy_node()
        except:
            pass
        try:
            if serial_node.ser:
                serial_node.ser.close()
        except:
            pass
        try:
            serial_node.destroy_node()
        except:
            pass
        try:
            goal_node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
