#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このプログラムは、ライトローバーをマウスで動かすためのノードです。
# 表示されたウィンドウ上で左ボタンを押している間だけ走行し、離すと停止します。
# ウィンドウ中央からのカーソルの位置で速度が決まります。
#   中央より上: 前進    中央より下: 後退
#   中央より左: 左旋回  中央より右: 右旋回
# 中央から離れるほど速くなり、ウィンドウの端で最大速度になります。

import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

# 中央付近の、この割合以内は 0 とみなす (ふらつき防止)
DEADZONE = 0.1


class Mouse(Node):
    def __init__(self):
        super().__init__('rover_mouse')
        self.declare_parameter('linear_scale', 0.2)
        self.declare_parameter('angular_scale', 1.2)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('window_size', 400)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.period_ms = int(1000.0 / float(self.get_parameter('publish_rate').value))
        self.size = int(self.get_parameter('window_size').value)
        self.publisher_ = self.create_publisher(Twist, 'rover_twist', 1)

        self.speed = Twist()
        self.pressed = False

        self.root = tk.Tk()
        self.root.title('LightRover mouse control')
        self.canvas = tk.Canvas(self.root, width=self.size, height=self.size, bg='white')
        self.canvas.pack()
        c = self.size / 2
        self.canvas.create_line(c, 0, c, self.size, fill='gray80')
        self.canvas.create_line(0, c, self.size, c, fill='gray80')
        self.canvas.create_text(c, 12, text='前進')
        self.canvas.create_text(c, self.size - 12, text='後退')
        self.canvas.create_text(24, c - 12, text='左旋回')
        self.canvas.create_text(self.size - 24, c - 12, text='右旋回')
        self.arrow = self.canvas.create_line(c, c, c, c, arrow=tk.LAST, width=3, fill='red')
        self.label = self.canvas.create_text(c, c + 24, text='')

        self.canvas.bind('<ButtonPress-1>', self.on_move)
        self.canvas.bind('<B1-Motion>', self.on_move)
        self.canvas.bind('<ButtonRelease-1>', self.on_release)
        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.root.after(self.period_ms, self.on_timer)

    def on_move(self, event):
        c = self.size / 2
        x = max(0, min(self.size, event.x))
        y = max(0, min(self.size, event.y))
        forward = apply_deadzone((c - y) / c)
        left = apply_deadzone((c - x) / c)
        self.speed.linear.x = forward * self.linear_scale
        self.speed.angular.z = left * self.angular_scale
        self.pressed = True
        self.canvas.coords(self.arrow, c, c, x, y)
        self.canvas.itemconfig(
            self.label,
            text='%.2f m/s  %.2f rad/s' % (self.speed.linear.x, self.speed.angular.z))

    def on_release(self, event):
        # ボタンを離したら停止 (停止指令は1回だけ出す)
        self.pressed = False
        self.speed = Twist()
        self.publisher_.publish(self.speed)
        c = self.size / 2
        self.canvas.coords(self.arrow, c, c, c, c)
        self.canvas.itemconfig(self.label, text='')

    def on_timer(self):
        if self.pressed:
            self.publisher_.publish(self.speed)
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(self.period_ms, self.on_timer)


def apply_deadzone(value):
    value = max(-1.0, min(1.0, value))
    if abs(value) < DEADZONE:
        return 0.0
    return value


def rover_mouse(args=None):
    # Ctrl+C で終了するときに停止指令を送れるよう、rclpy のシグナルハンドラは使わない
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    mouse = Mouse()
    mouse.get_logger().info('Mouse node start')
    try:
        mouse.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        mouse.publisher_.publish(Twist())
        mouse.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    rover_mouse()
