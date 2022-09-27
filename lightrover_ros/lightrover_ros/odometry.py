#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーのオドメトリを取得するためのノードです。

import rclpy
from rclpy.node import Node
import sys
from lightrover_interface.srv import *
import time
import math
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion ,TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
from nav_msgs.msg import Odometry

MS32_M_POS0 = 0x60
MS32_M_POS1 = 0x64

pre_count = [0.0, 0.0]

diff_count = [0, 0]

DIFF_COUNT_LIMIT = 1048575

pre_time = time.time()
diff_time = 0

WHEEL_CIRCUMFERENCE = 60.0*math.pi/1000
ENC_COUNTS_PER_TURN = 1188.024
ENC_PER_M = ENC_COUNTS_PER_TURN/WHEEL_CIRCUMFERENCE

ROVER_D = 0.143/2.0

x = 0.0
y = 0.0
th = 0.0

class OdometryManager(Node):
    def __init__(self):
        super().__init__('wrc201_odometry')
        self.read_enc = self.create_client(Wrc201Msg,'wrc201_i2c')
        while not self.read_enc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Sevice is not available')
        self.req = Wrc201Msg.Request()
        self.publisher_ = self.create_publisher(Odometry, 'odom', 50)
        self.rate = self.create_rate(100)

    def getEncVal(self):
        self.req.addr = MS32_M_POS0
        self.req.data = 0
        self.req.length = 4
        self.req.cmd = 'r'
        self.enc_a = self.read_enc.call_async(self.req)
        self.req.addr = MS32_M_POS1
        self.enc_b = self.read_enc.call_async(self.req)
    
def calSpeed(enc_val):
    global diff_time
    global pre_time
    global diff_count
    now_time = time.time()
    diff_time = now_time-pre_time
    pre_time = now_time

    global pre_count

    if enc_val is None:
        print('None')
        return None

    #以前のエンコーダ値と現在のエンコーダ値の差を算出
    for i in range(2):
        if(abs(enc_val[i]-pre_count[i]) < DIFF_COUNT_LIMIT):
            diff_count[i] = -1.0 * (enc_val[i]-pre_count[i])

    pre_count = enc_val

    #各タイヤの移動距離を算出
    distance = [float(diff_count[0])/ENC_PER_M,float(diff_count[1])/ENC_PER_M]
    #各タイヤの回転速度を算出
    speed = [distance[0]/diff_time, distance[1]/diff_time]

    #本体の直進・旋回速度を算出
    linear_x = ((speed[0] - speed[1])/2.0)
    angular_z = -1.0 * ((speed[0] + speed[1])/(2.0*ROVER_D))

    return linear_x, angular_z

def cal_odometry(vx, vth):
    global x,y,th
    delta_x = vx*math.cos(th)*diff_time
    delta_y = vx*math.sin(th)*diff_time
    delta_th = vth*diff_time

    x += delta_x
    y += delta_y
    th += delta_th


def lightrover_odometry(args=None):
    rclpy.init(args=args)

    odom_manager = OdometryManager()

    odom_manager.get_logger().info('Start odom manager')

    odom_br = TransformBroadcaster(odom_manager)

    odom_manager.getEncVal()

    while rclpy.ok():
        rclpy.spin_once(odom_manager)
        get_val = None
        if odom_manager.enc_a.done() and odom_manager.enc_b.done():
            get_val_a = odom_manager.enc_a.result().read_data
            get_val_b = odom_manager.enc_b.result().read_data
            get_val = calSpeed([get_val_a, get_val_b])       

        if get_val is None:
            continue
        else: 
            #odom_manager.get_logger().info('speed1 : %lf , speed2 : %lf' % ( get_val[0],get_val[1]))
            cal_odometry(get_val[0],get_val[1])

            now_time = odom_manager.get_clock().now().to_msg()

            odom_tf = TransformStamped()
            odom_tf.header.stamp = now_time
            odom_tf.header.frame_id = 'odom'
            odom_tf.child_frame_id = 'base_link'

            odom_quat = tf_transformations.quaternion_from_euler(0, 0, th)

            odom_tf.transform.translation.x = x
            odom_tf.transform.translation.y = y
            odom_tf.transform.translation.z = 0.0
            odom_tf.transform.rotation.x = odom_quat[0]
            odom_tf.transform.rotation.y = odom_quat[1]
            odom_tf.transform.rotation.z = odom_quat[2]
            odom_tf.transform.rotation.w = odom_quat[3]

            odom_br.sendTransform(odom_tf)

            odom = Odometry()
            odom.header.stamp = now_time

            odom.header.frame_id = "odom"

            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]

            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = get_val[0]
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = get_val[1]

            odom_manager.publisher_.publish(odom)

            odom_manager.rate.sleep()

            odom_manager.getEncVal()

if __name__=="__main__":
    lightrover_odometry() 


