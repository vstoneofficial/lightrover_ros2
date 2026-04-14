#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import struct
import time
import rclpy
from rclpy.node import Node
from lightrover_interface.srv import Wrc201Msg
from lightrover_ros.vs_wrc201_i2c import VsWrc201I2c


class Wrc201I2cServer(Node):
    ENC_ADDRS = (0x60, 0x64)
    MOTOR_CMD_ADDRS = (0x48, 0x4C)
    MOTOR_TRIGGER_ADDR = 0x11
    MOTOR_TRIGGER_VALUE = 0x03

    def __init__(self):
        super().__init__('wrc201_i2c_server')
        self.i2c = VsWrc201I2c(0x10)
        self.read_debug_counter = 0
        self.srv = None

        if not self.initialize_i2c():
            self.get_logger().error('I2C initialization failed, node stays alive for retry.')
        else:
            self.get_logger().info('I2C initialization success.')

        # Advertise the service only after initialization so clients don't
        # send motor setup commands before the controller is ready.
        self.srv = self.create_service(Wrc201Msg, 'wrc201_i2c', self.handle_wrc201_i2c)
        self.get_logger().info('Service is start')

    @staticmethod
    def to_signed_32(value: int) -> int:
        if value >= 0x80000000:
            return value - 0x100000000
        return value

    @staticmethod
    def pack_two_signed32(low_value: int, high_value: int) -> int:
        packed = struct.pack('<ii', int(low_value), int(high_value))
        return struct.unpack('<q', packed)[0]

    @staticmethod
    def unpack_two_signed32(value: int) -> tuple[int, int]:
        packed = struct.pack('<q', int(value))
        return struct.unpack('<ii', packed)

    def initialize_i2c(self, retries: int = 5, delay_sec: float = 0.2) -> bool:
        for i in range(retries):
            try:
                time.sleep(0.2)
                self.i2c.read_all()
                self.i2c.init_memmap(2.0)
                self.i2c.send_write_map()
                return True
            except OSError as e:
                self.get_logger().warn(f'I2C init failed ({i + 1}/{retries}): {e}')
                time.sleep(delay_sec)
            except Exception as e:
                self.get_logger().error(f'Unexpected I2C init error ({i + 1}/{retries}): {e}')
                time.sleep(delay_sec)
        return False

    def handle_wrc201_i2c(self, req, response):
        try:
            if req.cmd == "motor_w":
                right_cmd, left_cmd = self.unpack_two_signed32(req.data)
                self.i2c.write_4_byte(self.MOTOR_CMD_ADDRS[0], right_cmd)
                self.i2c.write_4_byte(self.MOTOR_CMD_ADDRS[1], left_cmd)
                self.i2c.write_1_byte(self.MOTOR_TRIGGER_ADDR, self.MOTOR_TRIGGER_VALUE)
                response.read_data = 1
                return response

            elif req.cmd == "enc_pair_r":
                self.i2c.read_multi_memmap(self.ENC_ADDRS[0], 8)
                enc_a = self.to_signed_32(int(self.i2c.read_s32map(self.ENC_ADDRS[0])))
                enc_b = self.to_signed_32(int(self.i2c.read_s32map(self.ENC_ADDRS[1])))
                response.read_data = self.pack_two_signed32(enc_a, enc_b)

                if self.read_debug_counter % 20 == 0:
                    self.get_logger().info(
                        f'I2C paired encoder read A={enc_a} B={enc_b}'
                    )
                self.read_debug_counter += 1
                return response

            elif req.cmd == "w":
                # メモリマップの特定アドレスへ書き込み
                if req.length == 4:
                    self.i2c.write_4_byte(req.addr, req.data)
                elif req.length == 2:
                    self.i2c.write_2_byte(req.addr, req.data)
                elif req.length == 1:
                    self.i2c.write_1_byte(req.addr, req.data)
                else:
                    self.get_logger().warn(f'Unsupported write length: {req.length}')
                    response.read_data = -1
                    return response

                response.read_data = 1
                return response

            elif req.cmd == "s":
                # 書き込みマップを送信
                self.i2c.send_write_map()
                response.read_data = 1
                return response

            elif req.cmd == "rm":
                # 全メモリマップ読み込み
                self.i2c.read_all()
                response.read_data = 1
                return response

            elif req.cmd == "r":
                # 特定アドレス読み込み
                self.i2c.read_multi_memmap(req.addr, req.length)

                if req.length == 4:
                    response.read_data = self.to_signed_32(
                        int(self.i2c.read_s32map(req.addr))
                    )
                elif req.length == 2:
                    response.read_data = int(self.i2c.read_s16map(req.addr))
                elif req.length == 1:
                    response.read_data = int(self.i2c.read_s8map(req.addr))
                else:
                    self.get_logger().warn(f'Unsupported read length: {req.length}')
                    response.read_data = 0

                if req.addr in self.ENC_ADDRS and self.read_debug_counter % 20 == 0:
                    self.get_logger().info(
                        f'I2C read addr=0x{req.addr:02X} length={req.length} data={response.read_data}'
                    )
                if req.addr in self.ENC_ADDRS:
                    self.read_debug_counter += 1

                return response

            else:
                self.get_logger().warn(f'Unknown I2C command: {req.cmd}')
                response.read_data = -1
                return response

        except IOError as e:
            self.get_logger().error(f'I2C IOError: {e}')
            response.read_data = -1
            return response
        except OSError as e:
            self.get_logger().error(f'I2C OSError: {e}')
            response.read_data = -1
            return response
        except Exception as e:
            self.get_logger().error(f'Unexpected service error: {e}')
            response.read_data = -1
            return response


def main(args=None):
    rclpy.init(args=args)
    node = Wrc201I2cServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def wrc201_i2c_server(args=None):
    main(args)


if __name__ == '__main__':
    main()
