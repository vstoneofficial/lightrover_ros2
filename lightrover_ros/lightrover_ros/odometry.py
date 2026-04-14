#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import struct
import rclpy
from rclpy.node import Node
from lightrover_interface.srv import Wrc201Msg
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry


MS32_M_POS0 = 0x60
MS32_M_POS1 = 0x64

DIFF_COUNT_LIMIT = 1048575
STARTUP_STABLE_COUNT_DELTA = 128
STARTUP_STABLE_SAMPLE_COUNT = 5

WHEEL_CIRCUMFERENCE = 60.0 * math.pi / 1000.0
ENC_COUNTS_PER_TURN = 1188.024
ENC_PER_M = ENC_COUNTS_PER_TURN / WHEEL_CIRCUMFERENCE

ROVER_D = 0.143 / 2.0
MAX_LINEAR_SPEED_MPS = 0.6
MAX_ANGULAR_SPEED_RADPS = 4.0


class OdometryManager(Node):
    def __init__(self):
        super().__init__('wrc201_odometry')

        self.read_enc = self.create_client(Wrc201Msg, 'wrc201_i2c')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.odom_br = TransformBroadcaster(self)

        self.pre_count = [0.0, 0.0]
        self.diff_count = [0.0, 0.0]
        self.encoder_initialized = False
        self.init_candidate = None
        self.init_stable_count = 0

        # time.time() ではなく ROS clock を使う
        self.last_stamp = self.get_clock().now()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.enc_future = None
        self.debug_counter = 0

        self.timer = self.create_timer(0.03, self.update_odom)

        if self.read_enc.wait_for_service(timeout_sec=1.0):
            self.get_enc_val()

    @staticmethod
    def unpack_two_signed32(value: int):
        packed = struct.pack('<q', int(value))
        return list(struct.unpack('<ii', packed))

    def make_request(self, addr: int = 0, length: int = 4, cmd: str = 'r'):
        req = Wrc201Msg.Request()
        req.addr = addr
        req.data = 0
        req.length = length
        req.cmd = cmd
        return self.read_enc.call_async(req)

    def get_enc_val(self):
        self.enc_future = self.make_request(MS32_M_POS0, 8, 'enc_pair_r')

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def cal_speed(self, enc_val, current_stamp):
        if enc_val is None:
            return None

        if not self.encoder_initialized:
            if self.init_candidate is None:
                self.init_candidate = list(enc_val)
                self.init_stable_count = 1
                self.last_stamp = current_stamp
                return 0.0, 0.0, 0.0

            if any(
                abs(enc_val[i] - self.init_candidate[i]) > STARTUP_STABLE_COUNT_DELTA
                for i in range(2)
            ):
                # Ignore unstable startup samples until encoder reads settle.
                self.init_candidate = list(enc_val)
                self.init_stable_count = 1
                self.last_stamp = current_stamp
                return 0.0, 0.0, 0.0

            self.init_stable_count += 1
            if self.init_stable_count < STARTUP_STABLE_SAMPLE_COUNT:
                self.last_stamp = current_stamp
                return 0.0, 0.0, 0.0

            self.pre_count = list(enc_val)
            self.last_stamp = current_stamp
            self.encoder_initialized = True
            self.init_candidate = None
            self.init_stable_count = 0
            return 0.0, 0.0, 0.0

        dt = (current_stamp - self.last_stamp).nanoseconds / 1e9
        self.last_stamp = current_stamp

        if dt <= 0.0:
            return None

        for i in range(2):
            if abs(enc_val[i] - self.pre_count[i]) < DIFF_COUNT_LIMIT:
                self.diff_count[i] = -1.0 * (enc_val[i] - self.pre_count[i])

        self.pre_count = enc_val

        distance = [
            float(self.diff_count[0]) / ENC_PER_M,
            float(self.diff_count[1]) / ENC_PER_M
        ]
        speed = [
            distance[0] / dt,
            distance[1] / dt
        ]

        # 元コード準拠
        linear_x = (speed[0] - speed[1]) / 2.0
        angular_z = -1.0 * ((speed[0] + speed[1]) / (2.0 * ROVER_D))

        if (
            abs(linear_x) > MAX_LINEAR_SPEED_MPS
            or abs(angular_z) > MAX_ANGULAR_SPEED_RADPS
        ):
            self.pre_count = list(enc_val)
            self.diff_count = [0.0, 0.0]
            self.get_logger().warn(
                'Discarding implausible odom sample '
                f'linear_x={linear_x:.4f} angular_z={angular_z:.4f} '
                f'raw_a={enc_val[0]} raw_b={enc_val[1]}'
            )
            return 0.0, 0.0, dt

        return linear_x, angular_z, dt

    def cal_odometry(self, vx, vth, dt):
        delta_x = vx * math.cos(self.th) * dt
        delta_y = vx * math.sin(self.th) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

    def publish_odom(self, linear_x, angular_z, stamp):
        odom_stamp = stamp.to_msg()
        odom_quat = self.yaw_to_quaternion(self.th)

        odom_tf = TransformStamped()
        odom_tf.header.stamp = odom_stamp
        odom_tf.header.frame_id = 'odom'
        odom_tf.child_frame_id = 'base_footprint'
        odom_tf.transform.translation.x = self.x
        odom_tf.transform.translation.y = self.y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation = odom_quat
        self.odom_br.sendTransform(odom_tf)

        odom = Odometry()
        odom.header.stamp = odom_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_z
        self.publisher_.publish(odom)

    def update_odom(self):
        current_stamp = self.get_clock().now()
        linear_x = 0.0
        angular_z = 0.0

        if not self.read_enc.wait_for_service(timeout_sec=0.01):
            if self.debug_counter % 50 == 0:
                self.get_logger().warn('Encoder service is not available')
            self.debug_counter += 1
            self.publish_odom(linear_x, angular_z, current_stamp)
            return

        if self.enc_future is None:
            self.get_enc_val()
            self.publish_odom(linear_x, angular_z, current_stamp)
            return

        if not self.enc_future.done():
            self.publish_odom(linear_x, angular_z, current_stamp)
            return

        res = self.enc_future.result()

        self.enc_future = None

        if res is None:
            if self.debug_counter % 50 == 0:
                self.get_logger().warn('Encoder future completed without a response')
            self.debug_counter += 1
            self.get_enc_val()
            self.publish_odom(linear_x, angular_z, current_stamp)
            return

        get_val_a, get_val_b = self.unpack_two_signed32(res.read_data)

        if self.debug_counter % 20 == 0:
            self.get_logger().info(
                f'Encoder raw values A={get_val_a} B={get_val_b}'
            )

        result = self.cal_speed([get_val_a, get_val_b], current_stamp)
        if result is None:
            if self.debug_counter % 50 == 0:
                self.get_logger().warn('Skipping odom update because dt <= 0')
            self.debug_counter += 1
            self.get_enc_val()
            self.publish_odom(linear_x, angular_z, current_stamp)
            return

        linear_x, angular_z, dt = result

        if self.debug_counter % 20 == 0:
            self.get_logger().info(
                f'Odom update linear_x={linear_x:.4f} angular_z={angular_z:.4f} '
                f'x={self.x:.4f} y={self.y:.4f} th={self.th:.4f}'
            )
        self.debug_counter += 1

        self.cal_odometry(linear_x, angular_z, dt)
        self.publish_odom(linear_x, angular_z, current_stamp)

        self.get_enc_val()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryManager()
    node.get_logger().info('Start odom manager')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def lightrover_odometry(args=None):
    main(args)


if __name__ == '__main__':
    main()
