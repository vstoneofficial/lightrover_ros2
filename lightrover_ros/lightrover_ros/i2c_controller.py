#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、VS-WRC201を制御するためのノードです。

import rclpy
from rclpy.node import Node
import lightrover_ros.vs_wrc201_i2c as vs_wrc201_i2c
import time

from lightrover_interface.srv import *

i2c = vs_wrc201_i2c.VsWrc201I2c(0x10)

class I2cController(Node):
    def __init__(self):
        super().__init__('wrc201_i2c_server')
        self.srv = self.create_service(Wrc201Msg, 'wrc201_i2c', self.handle_wrc201_i2c)
        self.get_logger().info('Service is start')

    def handle_wrc201_i2c(self, req, read):
        #self.get_logger().info('%d' % req.addr)
        if(req.cmd=="w"):
            #マイコンのメモリマップの特定アドレスを上書き
            try:
                if(req.length==4):#4byte
                    i2c.write_4_byte(req.addr,req.data)
                elif(req.length==2):#2byte
                    i2c.write_2_byte(req.addr,req.data)
                elif(req.length==1):#1byte
                    i2c.write_1_byte(req.addr,req.data)
                read.read_data=1
            except IOError as e:
                return None

            return read

        elif(req.cmd=="s"):
            #マイコンのメモリマップをすべて上書き
            try:
                i2c.send_write_map()
                read.read_data=1
            except IOError as e:
                return None

            return read

        elif(req.cmd=='rm'):
            #マイコンのメモリマップをすべて読み込み
            try:
                i2c.read_all()
                read.read_data=1
            except IOError as e:
                return None

            return read

        elif(req.cmd=="r"):
            #マイコンのメモリマップの特定アドレスを読み込み
            try:
                i2c.read_memmap(req.addr,req.length)
                if(req.length==4):#4byte
                    read.read_data=i2c.read_s32map(req.addr)
                elif(req.length==2):#2byte
                    read.read_data=i2c.read_s16map(req.addr)
                elif(req.length==1):#1byte
                    read.read_data=i2c.read_s8map(req.addr)
                else:
                    read.read_data=0
            except IOError as e:
                return None

            return read

def wrc201_i2c_server(args=None):
    rclpy.init(args=args)

    wrc201_i2c_server = I2cController()

    p_dev_addr = wrc201_i2c_server.declare_parameter('dev_addr', 0x10).value

    i2c.set_dev_addr(p_dev_addr)
    i2c.read_all()
    i2c.init_memmap(2.0)
    i2c.send_write_map()

    rclpy.spin(wrc201_i2c_server)

if __name__ == '__main__':
    wrc201_i2c_server()

