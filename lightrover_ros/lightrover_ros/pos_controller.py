#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーを位置制御するためのノードです。

import rclpy
from rclpy.node import Node
import sys
from lightrover_interface.srv import *
import time
import math
import lightrover_ros.vs_wrc201_motor as vs_wrc201_motor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#メモリマップアドレス
MU8_O_EN = 0x10
MU8_TRIG = 0x11
MS16_FB_PG0 = 0x20
MS16_FB_PG1 = 0x22

MS32_A_POS0 = 0x48
MS32_A_POS1 = 0x4c

MS16_T_OUT0 = 0x50
MS16_T_OUT1 = 0x52

MU16_FB_PCH0 = 0x30
MU16_FB_PCH1 = 0x32

linear_x = 0.0
angular_z = 0.0

current_v = [0.0, 0.0]
target_rover_v = [0.0, 0.0]

#車輪間距離の半分
ROVER_D = 0.143/2.0

motor_controller = vs_wrc201_motor.VsWrc201Motor()

class DriveMotor(Node):
    def __init__(self):
        super().__init__('pos_controller')
        self.write_msg = self.create_client(Wrc201Msg, 'wrc201_i2c')
        while not self.write_msg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sevice is not available')
        self.req = Wrc201Msg.Request()
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.cb_get_rover_v,
            1
            )
        self.drive_subscriber = self.create_subscription(
            Twist,
            'rover_twist',
            self.cb_set_target_v,
            1
            )

        # 速度指令のタイムアウト [s]
        # 最後の指令からこの時間を超えて新しい指令が来なければ目標速度を0にする
        # 0以下を指定するとタイムアウトを無効にする
        self.cmd_timeout = float(self.declare_parameter('cmd_timeout', 0.5).value)
        self.last_cmd_time = None
        self.cmd_timed_out = False

    def cb_get_rover_v(self, data):
        global linear_x, angular_z, current_v, ROVER_D, target_rover_v
        linear_x = data.twist.twist.linear.x
        angular_z = data.twist.twist.angular.z

        current_v[1] = (linear_x + ROVER_D * angular_z)
        current_v[0] = -1.0 * (linear_x - ROVER_D *angular_z)

        self.check_cmd_timeout()

        output = motor_controller.pos_controll(current_v, target_rover_v)
        self.drive_motor(output[0], output[1])

        time.sleep(0.05)
        
    def cb_set_target_v(self, data):
        global ROVER_D, target_rover_v

        target_rover_v[1] = (data.linear.x + ROVER_D * data.angular.z)
        target_rover_v[0] = -1.0 * (data.linear.x - ROVER_D * data.angular.z)

        self.last_cmd_time = time.monotonic()
        if self.cmd_timed_out:
            self.get_logger().info('Velocity command received again, resuming')
            self.cmd_timed_out = False

    def check_cmd_timeout(self):
        global target_rover_v

        if self.cmd_timeout <= 0.0 or self.last_cmd_time is None or self.cmd_timed_out:
            return
        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            # 停止指令(0)を受けた後に指令が途絶えるのは正常なので警告しない
            if target_rover_v[0] != 0.0 or target_rover_v[1] != 0.0:
                self.get_logger().warn(
                    'No velocity command for %.2f s, stopping the rover' % self.cmd_timeout)
            target_rover_v[0] = 0.0
            target_rover_v[1] = 0.0
            self.cmd_timed_out = True
    
    def set_req(self, addr, data, length, cmd):
        self.req.addr = addr
        self.req.data = data
        self.req.length = length
        self.req.cmd = cmd

        return self.req

    def drive_motor(self, r_speed, l_speed):
        self.write_msg.call_async(self.set_req(MS32_A_POS0,r_speed,4,'w'))
        self.write_msg.call_async(self.set_req(MS32_A_POS1,l_speed,4,'w'))
        self.write_msg.call_async(self.set_req(MU8_TRIG,0x03,1,'w'))
        
def pos_cntrl(args=None):
    rclpy.init(args=args)

    pos_controller = DriveMotor()

    pos_controller.get_logger().info('Start POS Controll')

    pos_controller.write_msg.call_async(pos_controller.set_req(MU8_O_EN,0x00,1,'w'))
    pos_controller.write_msg.call_async(pos_controller.set_req(MU8_TRIG,0x0c,1,'w'))
    pos_controller.write_msg.call_async(pos_controller.set_req(MS16_FB_PG0,0x0080,2,'w'))
    pos_controller.write_msg.call_async(pos_controller.set_req(MS16_FB_PG1,0x0080,2,'w'))
    pos_controller.write_msg.call_async(pos_controller.set_req(MU16_FB_PCH0,0x09C4,2,'w'))
    pos_controller.write_msg.call_async(pos_controller.set_req(MU16_FB_PCH1,0x09C4,2,'w'))
    pos_controller.write_msg.call_async(pos_controller.set_req(MU8_O_EN,0x03,1,'w'))

    rclpy.spin(pos_controller)

if __name__=='__main__':
    pos_cntrl()
