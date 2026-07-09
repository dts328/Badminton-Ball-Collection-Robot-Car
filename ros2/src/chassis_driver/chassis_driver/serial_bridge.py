#!/usr/bin/env python3
"""
Chassis Serial Bridge Node
串口通信桥接节点 - 与STM32底盘通信 (二进制协议)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, UInt8MultiArray
import serial
import struct
import threading
import time


# 二进制协议常量
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
MODE_MECANUM = 0x00
MODE_ARM = 0x01
RX_FRAME_LEN = 11   # 接收帧长度
TX_FRAME_LEN = 26   # 发送帧长度


def calc_checksum(buf):
    """计算XOR校验"""
    checksum = 0
    for b in buf[:-2]:  # 不包括校验位和帧尾
        checksum ^= b
    return checksum


def build_mecanum_frame(vx, vy, vz):
    """构建底盘控制帧 (11字节)
    
    Args:
        vx: X轴速度 (m/s)
        vy: Y轴速度 (m/s)
        vz: Z轴角速度 (rad/s)
    
    Returns:
        bytes: 11字节二进制帧
    """
    # 转换为mm/s并转为int16
    x = int(vx * 1000)
    y = int(vy * 1000)
    z = int(vz * 1000)
    
    # 限幅
    x = max(-32768, min(32767, x))
    y = max(-32768, min(32767, y))
    z = max(-32768, min(32767, z))
    
    buf = bytearray(RX_FRAME_LEN)
    buf[0] = FRAME_HEADER
    buf[1] = MODE_MECANUM
    buf[2] = 0x00  # 保留
    buf[3:5] = struct.pack('>h', x)  # 大端序int16
    buf[5:7] = struct.pack('>h', y)
    buf[7:9] = struct.pack('>h', z)
    buf[9] = calc_checksum(buf)
    buf[10] = FRAME_TAIL
    return bytes(buf)


def build_arm_frame(j1, j2, j3):
    """构建机械臂控制帧 (11字节)
    
    Args:
        j1: 关节1角度 (度)
        j2: 关节2角度 (度)
        j3: 关节3角度 (度)
    
    Returns:
        bytes: 11字节二进制帧
    """
    # 转换为0.01度并转为int16
    j1_raw = int(j1 * 100)
    j2_raw = int(j2 * 100)
    j3_raw = int(j3 * 100)
    
    # 限幅
    j1_raw = max(-18000, min(18000, j1_raw))
    j2_raw = max(-18000, min(18000, j2_raw))
    j3_raw = max(-18000, min(18000, j3_raw))
    
    buf = bytearray(RX_FRAME_LEN)
    buf[0] = FRAME_HEADER
    buf[1] = MODE_ARM
    buf[2] = 0x00
    buf[3:5] = struct.pack('>h', j1_raw)
    buf[5:7] = struct.pack('>h', j2_raw)
    buf[7:9] = struct.pack('>h', j3_raw)
    buf[9] = calc_checksum(buf)
    buf[10] = FRAME_TAIL
    return bytes(buf)


def parse_status_frame(buf):
    """解析状态帧 (26字节)
    
    帧格式:
    [0x7B][flags][A_H][A_L][B_H][B_L][C_H][C_L][D_H][D_L]
    [accX_H][accX_L][accY_H][accY_L][accZ_H][accZ_L]
    [gyroX_H][gyroX_L][gyroY_H][gyroY_L][gyroZ_H][gyroZ_L]
    [bat_H][bat_L][checksum][0x7D]
    
    Returns:
        dict: 解析后的数据，None表示解析失败
    """
    if len(buf) != TX_FRAME_LEN:
        return None
    
    # 验证帧头帧尾
    if buf[0] != FRAME_HEADER or buf[25] != FRAME_TAIL:
        return None
    
    # 验证校验
    expected_checksum = 0
    for i in range(24):
        expected_checksum ^= buf[i]
    if buf[24] != expected_checksum:
        return None
    
    # 解析数据
    flags = buf[1]
    enc_a = struct.unpack('>h', buf[2:4])[0] / 1000.0   # m/s
    enc_b = struct.unpack('>h', buf[4:6])[0] / 1000.0
    enc_c = struct.unpack('>h', buf[6:8])[0] / 1000.0
    enc_d = struct.unpack('>h', buf[8:10])[0] / 1000.0
    
    # IMU数据 (原始值)
    accel_x = struct.unpack('>h', buf[10:12])[0]
    accel_y = struct.unpack('>h', buf[12:14])[0]
    accel_z = struct.unpack('>h', buf[14:16])[0]
    gyro_x = struct.unpack('>h', buf[16:18])[0]
    gyro_y = struct.unpack('>h', buf[18:20])[0]
    gyro_z = struct.unpack('>h', buf[20:22])[0]
    
    # 电池电压
    battery = struct.unpack('>H', buf[22:24])[0] / 1000.0  # V
    
    return {
        'flags': flags,
        'encoders': [enc_a, enc_b, enc_c, enc_d],
        'accel': [accel_x, accel_y, accel_z],
        'gyro': [gyro_x, gyro_y, gyro_z],
        'battery': battery,
        'stop_flag': bool(flags & 0x01)
    }


class SerialBridge(Node):
    """串口桥接节点"""
    
    def __init__(self):
        super().__init__('chassis_serial_bridge')
        
        # 参数声明
        self.declare_parameter('port_name', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        
        # 获取参数
        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # 串口初始化
        self.serial_port = None
        self.port_name = port_name
        self.baudrate = baudrate
        self.connect_serial()
        
        # 接收缓冲区
        self.rx_buf = bytearray()
        
        # 订阅速度指令
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 发布状态 (二进制)
        self.status_pub = self.create_publisher(UInt8MultiArray, '/chassis_status', 10)
        self.voltage_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        
        # 接收线程
        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        self.get_logger().info(f'Chassis Serial Bridge started: {port_name}@{baudrate}')
        self.get_logger().info(f'Protocol: Binary (0x{FRAME_HEADER:02X}...0x{FRAME_TAIL:02X})')
    
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.get_logger().info(f'Serial port connected: {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect serial: {e}')
            self.serial_port = None
    
    def cmd_vel_callback(self, msg):
        """速度指令回调 - 发送二进制帧"""
        if self.serial_port is None:
            return
        
        # 构建二进制帧
        frame = build_mecanum_frame(msg.linear.x, msg.linear.y, msg.angular.z)
        
        try:
            self.serial_port.write(frame)
            self.get_logger().debug(f'Sent frame: {frame.hex()}')
        except Exception as e:
            self.get_logger().error(f'Send failed: {e}')
            self.connect_serial()
    
    def receive_loop(self):
        """接收线程 - 解析二进制帧"""
        while rclpy.ok():
            if self.serial_port is None:
                time.sleep(0.1)
                continue
            
            try:
                if self.serial_port.in_waiting > 0:
                    # 读取所有可用数据
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    self.rx_buf.extend(data)
                    
                    # 查找完整帧
                    while len(self.rx_buf) >= TX_FRAME_LEN:
                        # 查找帧头
                        header_idx = self.rx_buf.find(FRAME_HEADER)
                        if header_idx == -1:
                            self.rx_buf.clear()
                            break
                        
                        # 移除帧头前的数据
                        if header_idx > 0:
                            self.rx_buf = self.rx_buf[header_idx:]
                        
                        # 检查是否有完整帧
                        if len(self.rx_buf) >= TX_FRAME_LEN:
                            # 提取一帧
                            frame = bytes(self.rx_buf[:TX_FRAME_LEN])
                            self.rx_buf = self.rx_buf[TX_FRAME_LEN:]
                            
                            # 解析帧
                            self.parse_frame(frame)
                        else:
                            break
                            
            except Exception as e:
                self.get_logger().debug(f'Receive error: {e}')
                time.sleep(0.01)
    
    def parse_frame(self, frame):
        """解析状态帧"""
        status = parse_status_frame(frame)
        
        if status is not None:
            # 发布状态
            msg = UInt8MultiArray()
            msg.data = list(frame)
            self.status_pub.publish(msg)
            
            # 发布电压
            voltage = Float32()
            voltage.data = status['battery']
            self.voltage_pub.publish(status['battery'])
            
            self.get_logger().debug(
                f'Status: enc={status["encoders"]}, bat={status["battery"]:.2f}V'
            )
        else:
            self.get_logger().warn('Invalid frame received')
    
    def send_arm_command(self, j1, j2, j3):
        """发送机械臂指令"""
        if self.serial_port is None:
            return False
        
        frame = build_arm_frame(j1, j2, j3)
        
        try:
            self.serial_port.write(frame)
            return True
        except Exception as e:
            self.get_logger().error(f'Arm send failed: {e}')
            return False
    
    def destroy_node(self):
        """销毁节点"""
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
