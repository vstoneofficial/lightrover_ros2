#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このプログラムは、ライトローバーをキーボードで動かすためのノードです。
# 端末上でキーを押している間だけ走行し、キーを離すと停止します。
#
#   w / ↑ : 前進        s / ↓ : 後退
#   a / ← : 左旋回      d / → : 右旋回
#   スペース : 停止     Ctrl+C : 終了
#
# キー入力を受け取るため、ros2 run で端末から直接起動してください。
#   ros2 run lightrover_ros rover_keyboard

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

HELP = """
---------------------------------------
 ライトローバー キーボード操作
---------------------------------------
   w / ↑ : 前進      s / ↓ : 後退
   a / ← : 左旋回    d / → : 右旋回
   スペース : 停止   Ctrl+C : 終了
 キーを押している間だけ走行します
---------------------------------------
"""

# キー -> (前後方向, 旋回方向)
KEY_BINDINGS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    '\x1b[A': (1.0, 0.0),   # ↑
    '\x1b[B': (-1.0, 0.0),  # ↓
    '\x1b[D': (0.0, 1.0),   # ←
    '\x1b[C': (0.0, -1.0),  # →
    ' ': (0.0, 0.0),
}


class Keyboard(Node):
    def __init__(self):
        super().__init__('rover_keyboard')
        self.declare_parameter('linear_scale', 0.1)
        self.declare_parameter('angular_scale', 0.8)
        self.declare_parameter('publish_rate', 20.0)
        # 端末はキーを離したことを通知しないため、キー入力が途切れたら停止とみなす。
        # キーを押し続けると、最初のリピートまでは 0.5 秒前後空き、その後は
        # 数十ミリ秒間隔で入力が届く。そこでリピートが始まるまでは initial_key_timeout、
        # リピート中は key_timeout で判定し、キーを離したらすぐ止まるようにする。
        self.declare_parameter('initial_key_timeout', 0.7)
        self.declare_parameter('key_timeout', 0.15)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.period = 1.0 / float(self.get_parameter('publish_rate').value)
        self.initial_key_timeout = float(self.get_parameter('initial_key_timeout').value)
        self.key_timeout = float(self.get_parameter('key_timeout').value)
        self.publisher_ = self.create_publisher(Twist, 'rover_twist', 1)

    def publish(self, linear, angular):
        speed = Twist()
        speed.linear.x = linear * self.linear_scale
        speed.angular.z = angular * self.angular_scale
        self.publisher_.publish(speed)


def read_key(timeout):
    """timeout 秒以内に押されたキーを返す。押されなければ None。"""
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    key = sys.stdin.read(1)
    if key == '\x1b':
        # 矢印キーは ESC [ A のような3文字で届く
        while select.select([sys.stdin], [], [], 0.01)[0] and len(key) < 3:
            key += sys.stdin.read(1)
    return key


def rover_keyboard(args=None):
    if not sys.stdin.isatty():
        print('rover_keyboard は端末から ros2 run で起動してください。')
        return

    # Ctrl+C で終了するときに停止指令を送れるよう、rclpy のシグナルハンドラは使わない
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    keyboard = Keyboard()
    print(HELP)

    settings = termios.tcgetattr(sys.stdin)
    command = (0.0, 0.0)
    last_key = None
    last_key_time = 0.0
    moving = False
    repeating = False
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(keyboard.period)
            if key == '\x03':  # Ctrl+C
                break
            if key in KEY_BINDINGS:
                # 走行中に同じキーが再び届いたら、キーリピート中とみなす
                repeating = moving and key == last_key
                command = KEY_BINDINGS[key]
                last_key = key
                last_key_time = time.monotonic()
                moving = command != (0.0, 0.0)
                keyboard.publish(*command)
            elif moving:
                timeout = keyboard.key_timeout if repeating else keyboard.initial_key_timeout
                if time.monotonic() - last_key_time > timeout:
                    # キーが離されたので停止 (停止指令は1回だけ出す)
                    moving = False
                    keyboard.publish(0.0, 0.0)
                else:
                    keyboard.publish(*command)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        keyboard.publish(0.0, 0.0)
        keyboard.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    rover_keyboard()
