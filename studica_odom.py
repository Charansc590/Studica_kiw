# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Imu
# import tf_transformations
# import tf2_ros
# from gpiozero import RotaryEncoder
# import serial
# import struct
# from time import time
# import math


# class EncoderIMUFused(Node):
#     def __init__(self):
#         super().__init__('encoder_imu_fused')

#         # ================= SERIAL =================
#         self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
#         self.packet_size = 48   # 12 floats

#         # ================= ROBOT PARAMETERS =================
#         self.CPR = 135
#         self.WHEEL_DIAM = 0.09
#         self.CIRC = math.pi * self.WHEEL_DIAM
#         self.R = 0.18

#         self.THETA_FL = math.radians(30)
#         self.THETA_FR = math.radians(150)
#         self.THETA_B  = math.radians(270)

#         # ================= ENCODERS =================
#         self.enc_fl = RotaryEncoder(5, 25, max_steps=0)
#         self.enc_fr = RotaryEncoder(23, 24, max_steps=0)
#         self.enc_b  = RotaryEncoder(17, 27, max_steps=0)

#         self.prev_steps = {'fl': 0, 'fr': 0, 'b': 0}
#         self.prev_time = time()

#         # ================= POSE =================
#         self.x = 0.0
#         self.y = 0.0
#         self.th = 0.0   # Will come from IMU

#         # ================= ROS PUB =================
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.dis_pub  = self.create_publisher(Float32MultiArray, '/dis_data', 10)
#         self.imu_pub  = self.create_publisher(Imu, '/imu', 10)

#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

#         self.timer = self.create_timer(0.02, self.update)  # 50 Hz

#         self.get_logger().info("✅ Encoder + IMU fused odom started")

#     # --------------------------------------------------
#     def steps_to_velocity(self, dsteps, dt):
#         return (dsteps / self.CPR) * self.CIRC / dt

#     # --------------------------------------------------
#     def read_serial_packet(self):

#         if self.ser.in_waiting >= self.packet_size:
#             data = self.ser.read(self.packet_size)
#             values = struct.unpack('12f', data)

#             yaw = values[0]
#             wz_imu = values[1]

#             ax, ay, az = values[2], values[3], values[4]
#             gx, gy, gz = values[5], values[6], values[7]

#             ir1, ir2 = values[8], values[9]
#             us1, us2 = values[10], values[11]

#             return yaw, wz_imu, ax, ay, az, gx, gy, gz, ir1, ir2, us1, us2

#         return None

#     # --------------------------------------------------
#     def update(self):

#         now = time()
#         dt = now - self.prev_time
#         if dt <= 0:
#             return

#         # ================= READ IMU =================
#         serial_data = self.read_serial_packet()
#         if serial_data is None:
#             return

#         yaw, wz_imu, ax, ay, az, gx, gy, gz, ir1, ir2, us1, us2 = serial_data

#         # Use IMU yaw directly
#         self.th = yaw

#         # ================= ENCODERS =================
#         steps = {
#             'fl': self.enc_fl.steps,
#             'fr': self.enc_fr.steps,
#             'b':  self.enc_b.steps
#         }

#         v = {}
#         for k in steps:
#             ds = steps[k] - self.prev_steps[k]
#             v[k] = self.steps_to_velocity(ds, dt)
#             self.prev_steps[k] = steps[k]

#         w1 = v['fl']
#         w2 = v['fr']
#         w3 = v['b']

#         vx = (2.0 / 3.0) * (
#             math.cos(self.THETA_FL) * w1 +
#             math.cos(self.THETA_FR) * w2 +
#             math.cos(self.THETA_B ) * w3
#         )

#         vy = (2.0 / 3.0) * (
#             math.sin(self.THETA_FL) * w1 +
#             math.sin(self.THETA_FR) * w2 +
#             math.sin(self.THETA_B ) * w3
#         )

#         wz = (1.0 / (3.0 * self.R)) * (w1 + w2 + w3)

#         # ================= POSITION INTEGRATION =================
#         dx = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
#         dy = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt

#         self.x += dx
#         self.y += dy

#         # ================= PUBLISH ODOM =================
#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y

#         q = tf_transformations.quaternion_from_euler(0, 0, self.th)
#         odom.pose.pose.orientation.x = q[0]
#         odom.pose.pose.orientation.y = q[1]
#         odom.pose.pose.orientation.z = q[2]
#         odom.pose.pose.orientation.w = q[3]

#         odom.twist.twist.linear.x = vx
#         odom.twist.twist.linear.y = vy
#         odom.twist.twist.angular.z = wz

#         self.odom_pub.publish(odom)

#         # ================= TF =================
#         t = TransformStamped()
#         t.header.stamp = odom.header.stamp
#         t.header.frame_id = 'odom'
#         t.child_frame_id = 'base_link'
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]
#         self.tf_broadcaster.sendTransform(t)

#         # ================= PUBLISH IMU =================
#         imu_msg = Imu()
#         imu_msg.header = odom.header
#         imu_msg.orientation.x = q[0]
#         imu_msg.orientation.y = q[1]
#         imu_msg.orientation.z = q[2]
#         imu_msg.orientation.w = q[3]
#         imu_msg.angular_velocity.z = gz
#         imu_msg.linear_acceleration.x = ax
#         imu_msg.linear_acceleration.y = ay
#         imu_msg.linear_acceleration.z = az
#         self.imu_pub.publish(imu_msg)

#         # ================= PUBLISH DISTANCE =================
#         dis_msg = Float32MultiArray()
#         dis_msg.data = [ir1, ir2, us1, us2]
#         self.dis_pub.publish(dis_msg)

#         # DEBUG
#         print(
#             f"x={self.x:.3f} | y={self.y:.3f} | "
#             f"th={math.degrees(self.th)%360:.2f} deg | "
#             f"vx={vx:.3f} | vy={vy:.3f}"
#         )

#         self.prev_time = now


# def main(args=None):
#     rclpy.init(args=args)
#     node = EncoderIMUFused()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import tf_transformations
import tf2_ros
from gpiozero import RotaryEncoder
import serial
import struct
import time
import math


class EncoderIMUFused(Node):

    def __init__(self):
        super().__init__('encoder_imu_fused')

        # ================= SERIAL =================
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # 🔥 FORCE ESP32 RESET
        self.ser.setDTR(False)
        time.sleep(0.2)
        self.ser.setDTR(True)
        time.sleep(2.0)   # wait for ESP to reboot

        self.packet_size = 48  # 12 floats
        self.header_size = 2   # 0xAA 0x55

        # ================= ROBOT PARAMETERS =================
        self.CPR = 390
        self.WHEEL_DIAM = 0.056   #0.065
        self.CIRC = math.pi * self.WHEEL_DIAM
        self.R = 0.27

        self.THETA_FL = math.radians(30)
        self.THETA_FR = math.radians(150)
        self.THETA_B  = math.radians(270)

        # ================= ENCODERS =================
        self.enc_fl = RotaryEncoder(5, 25, max_steps=0)
        self.enc_fr = RotaryEncoder(23, 24, max_steps=0)
        self.enc_b  = RotaryEncoder(17, 27, max_steps=0)

        self.prev_steps = {'fl': 0, 'fr': 0, 'b': 0}
        self.prev_time = time.time()

        # ================= POSE =================
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # ================= ROS PUB =================
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.dis_pub  = self.create_publisher(Float32MultiArray, '/dis_data', 10)
        self.imu_pub  = self.create_publisher(Imu, '/imu', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info("✅ Encoder + IMU fused node started (ESP auto-reset enabled)")

    # --------------------------------------------------
    def steps_to_velocity(self, dsteps, dt):
        return (dsteps / self.CPR) * self.CIRC / dt

    # --------------------------------------------------
    def read_serial_packet(self):

        while self.ser.in_waiting >= (self.header_size + self.packet_size):

            if self.ser.read(1) == b'\xAA':
                if self.ser.read(1) == b'\x55':

                    data = self.ser.read(self.packet_size)

                    if len(data) == self.packet_size:
                        values = struct.unpack('12f', data)
                        return values

        return None

    # --------------------------------------------------
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

        # ================= INTEGRATE YAW FROM IMU =================
        self.th -= wz_imu * dt

        if self.th > math.pi:
            self.th -= 2 * math.pi
        if self.th < -math.pi:
            self.th += 2 * math.pi

        # ================= ENCODERS =================
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

        # ================= PUBLISH ODOM =================
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

        # ================= TF =================
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

        # ================= IMU =================
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

        # ================= DISTANCE =================
        dis_msg = Float32MultiArray()
        dis_msg.data = [ir1, ir2, us1, us2]
        self.dis_pub.publish(dis_msg)

        print(f"x={self.x:.3f} | y={self.y:.3f} | th={math.degrees(self.th)%360:.2f}")

        self.prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = EncoderIMUFused()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
