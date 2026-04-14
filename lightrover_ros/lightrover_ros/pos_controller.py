#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
import rclpy
from rclpy.node import Node
from lightrover_interface.srv import Wrc201Msg
import lightrover_ros.vs_wrc201_motor as vs_wrc201_motor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# メモリマップアドレス
MU8_O_EN = 0x10
MU8_TRIG = 0x11
MS16_FB_PG0 = 0x20
MS16_FB_PG1 = 0x22

MS32_A_POS0 = 0x48
MS32_A_POS1 = 0x4c

MU16_FB_PCH0 = 0x30
MU16_FB_PCH1 = 0x32

linear_x = 0.0
angular_z = 0.0

current_v = [0.0, 0.0]
target_rover_v = [0.0, 0.0]

ROVER_D = 0.143 / 2.0

motor_controller = vs_wrc201_motor.VsWrc201Motor()


class DriveMotor(Node):
    def __init__(self):
        super().__init__('pos_controller')
        self.debug_counter = 0
        self.pending_write = None

        self.write_msg = self.create_client(Wrc201Msg, 'wrc201_i2c')
        while not self.write_msg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service is not available')

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.cb_get_rover_v,
            10
        )

        self.drive_subscriber = self.create_subscription(
            Twist,
            'rover_twist',
            self.cb_set_target_v,
            10
        )
        
        self.nav_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cb_set_target_v,
            10
        )

        self.timer = self.create_timer(0.03, self.control_loop)

    def make_req(self, addr, data, length, cmd):
        req = Wrc201Msg.Request()
        req.addr = int(addr)
        req.data = int(data)
        req.length = int(length)
        req.cmd = cmd
        return req

    def send_async(self, addr, data, length, cmd):
        req = self.make_req(addr, data, length, cmd)
        return self.write_msg.call_async(req)

    @staticmethod
    def pack_two_signed32(low_value: int, high_value: int) -> int:
        packed = struct.pack('<ii', int(low_value), int(high_value))
        return struct.unpack('<q', packed)[0]

    def has_pending_write(self):
        return self.pending_write is not None and not self.pending_write.done()

    def cb_get_rover_v(self, data):
        global linear_x, angular_z, current_v, ROVER_D

        linear_x = data.twist.twist.linear.x
        angular_z = data.twist.twist.angular.z

        current_v[1] = (linear_x + ROVER_D * angular_z)
        current_v[0] = -1.0 * (linear_x - ROVER_D * angular_z)

    def cb_set_target_v(self, data):
        global ROVER_D, target_rover_v

        target_rover_v[1] = (data.linear.x + ROVER_D * data.angular.z)
        target_rover_v[0] = -1.0 * (data.linear.x - ROVER_D * data.angular.z)

    def control_loop(self):
        output = motor_controller.pos_controll(current_v, target_rover_v)
        if self.debug_counter % 20 == 0:
            self.get_logger().info(
                'Control current_v=(%.3f, %.3f) target_v=(%.3f, %.3f) output=(%d, %d)'
                % (
                    current_v[0], current_v[1],
                    target_rover_v[0], target_rover_v[1],
                    output[0], output[1],
                )
            )
        if self.has_pending_write():
            if self.debug_counter % 20 == 0:
                self.get_logger().info('Skipping motor write because previous combined I2C write is still pending')
            self.debug_counter += 1
            return
        self.debug_counter += 1
        self.drive_motor(output[0], output[1])

    def drive_motor(self, r_speed, l_speed):
        packed_output = self.pack_two_signed32(r_speed, l_speed)
        self.pending_write = self.send_async(MS32_A_POS0, packed_output, 8, 'motor_w')

    def initialize_motor(self):
        self.send_async(MU8_O_EN, 0x00, 1, 'w')
        self.send_async(MU8_TRIG, 0x0c, 1, 'w')
        self.send_async(MS16_FB_PG0, 0x0080, 2, 'w')
        self.send_async(MS16_FB_PG1, 0x0080, 2, 'w')
        self.send_async(MU16_FB_PCH0, 0x09C4, 2, 'w')
        self.send_async(MU16_FB_PCH1, 0x09C4, 2, 'w')
        self.send_async(MU8_O_EN, 0x03, 1, 'w')


def main(args=None):
    rclpy.init(args=args)

    pos_controller = DriveMotor()
    pos_controller.get_logger().info('Start POS Controll')
    pos_controller.initialize_motor()

    try:
        rclpy.spin(pos_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pos_controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def pos_cntrl(args=None):
    main(args)


if __name__ == '__main__':
    main()
