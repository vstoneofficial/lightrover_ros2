#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーをゲームパッドで動かすためのノードです。

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

speed = Twist()

class GamePad(Node):
    def __init__(self):
        super().__init__('rover_gamepad')
        self.publisher_ = self.create_publisher(Twist, 'rover_twist', 1)
        self.subscription_ = self.create_subscription(
            Joy,
            'joy',
            self.callback,
            1
        )
        self.rate = self.create_rate(20)

    def callback(self, data):
        global speed

        speed.linear.x = data.axes[1]*0.1
        speed.angular.z = data.axes[2]*2.0

        self.publisher_.publish(speed)

def rover_gamepad(args=None):
    global speed

    rclpy.init(args=args)

    game_pad = GamePad()

    game_pad.get_logger().info('Game pad node start')

    rclpy.spin(game_pad)

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
