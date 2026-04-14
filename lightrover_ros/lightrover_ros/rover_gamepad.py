#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーをゲームパッドで動かすためのノードです。

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

speed = Twist()

class GamePad(Node):
    def __init__(self):
        super().__init__('rover_gamepad')
        self.declare_parameter('linear_scale', 0.3)
        self.declare_parameter('angular_scale', 2.0)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.publisher_ = self.create_publisher(Twist, 'rover_twist', 1)
        self.subscription_ = self.create_subscription(
            Joy,
            'joy',
            self.callback,
            1
        )
        self.debug_counter = 0

    def callback(self, data):
        global speed

        speed.linear.x = data.axes[1] * self.linear_scale
        speed.angular.z = data.axes[2] * self.angular_scale

        self.debug_counter += 1
        if self.debug_counter % 20 == 0:
            axes = ', '.join(f'{value:.3f}' for value in data.axes)
            buttons = ', '.join(str(value) for value in data.buttons)
            self.get_logger().info(
                f'Joy axes=[{axes}] buttons=[{buttons}] -> '
                f'rover_twist linear.x={speed.linear.x:.3f} angular.z={speed.angular.z:.3f}'
            )

        self.publisher_.publish(speed)

        time.sleep(0.05)

def rover_gamepad(args=None):
    global speed

    rclpy.init(args=args)

    game_pad = GamePad()

    game_pad.get_logger().info('Game pad node start')

    rclpy.spin(game_pad)

    game_pad.destroy_node()
    rclpy.shutdown()

    '''

    while rclpy.ok():
        rclpy.spin_once(game_pad)
        game_pad.get_logger().info('publish')
        #game_pad.publisher_.publish(speed)
        game_pad.get_logger().info('publish2')
        game_pad.rate.sleep()
        game_pad.get_logger().info('publish3')

    '''

if __name__=='__main__':
    rover_gamepad()
