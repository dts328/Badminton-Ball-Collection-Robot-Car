#!/usr/bin/env python3
"""
LD06 Lidar Driver Node
乐动LD06激光雷达驱动节点
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import serial
import struct
import threading
import math
import time


class LDLidarNode(Node):
    """LD06激光雷达驱动节点"""
    
    # LD06协议常量
    HEADER = 0x54
    VERLEN = 0x2C
    POINT_PER_PACK = 12
    
    def __init__(self):
        super().__init__('ldlidar_node')
        
        # 参数声明
        self.declare_parameter('port_name', '/dev/ttyACM2')
        self.declare_parameter('port_baudrate', 230400)
        self.declare_parameter('topic_name', '/scan')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_frequency', 10.0)
        
        # 获取参数
        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('port_baudrate').get_parameter_value().integer_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.frame_id = frame_id
        
        # 发布者
        self.scan_pub = self.create_publisher(LaserScan, topic_name, 10)
        
        # 串口
        self.serial_port = None
        self.port_name = port_name
        self.baudrate = baudrate
        self.connect_serial()
        
        # 扫描数据
        self.scan_data = [0.0] * 360
        self.scan_ready = False
        
        # 接收线程
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        # 发布定时器
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10Hz
        
        self.get_logger().info(f'LD06 Lidar started: {port_name}@{baudrate}')
    
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.get_logger().info(f'Serial connected: {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Connect failed: {e}')
            self.serial_port = None
    
    def receive_loop(self):
        """接收线程"""
        while self.running and rclpy.ok():
            if self.serial_port is None:
                time.sleep(0.1)
                self.connect_serial()
                continue
            
            try:
                # 读取数据包
                data = self.read_packet()
                if data:
                    self.parse_packet(data)
            except Exception as e:
                self.get_logger().debug(f'Receive error: {e}')
                time.sleep(0.001)
    
    def read_packet(self):
        """读取一个数据包"""
        # 查找包头
        while self.running:
            byte = self.serial_port.read(1)
            if len(byte) == 0:
                return None
            if byte[0] == self.HEADER:
                break
        
        # 读取长度
        verlen = self.serial_port.read(1)
        if len(verlen) == 0 or verlen[0] != self.VERLEN:
            return None
        
        # 读取剩余数据
        # 1(header) + 1(verlen) + 2(speed) + 2(start_angle) + 36(data) + 2(end_angle) + 2(timestamp) + 1(crc) = 47
        remaining = self.serial_port.read(45)
        if len(remaining) < 45:
            return None
        
        return remaining
    
    def parse_packet(self, data):
        """解析数据包"""
        try:
            # 解析速度
            speed = struct.unpack('<H', data[0:2])[0]
            
            # 解析起始角度
            start_angle = struct.unpack('<H', data[2:4])[0] / 100.0
            
            # 解析12个测量点
            points = []
            for i in range(self.POINT_PER_PACK):
                offset = 4 + i * 3
                distance = struct.unpack('<H', data[offset:offset+2])[0] / 1000.0  # mm -> m
                intensity = data[offset+2]
                points.append((distance, intensity))
            
            # 解析结束角度
            end_angle = struct.unpack('<H', data[38:40])[0] / 100.0
            
            # 计算角度步长
            if end_angle < start_angle:
                end_angle += 360.0
            angle_step = (end_angle - start_angle) / (self.POINT_PER_PACK - 1)
            
            # 填充扫描数据
            for i, (distance, intensity) in enumerate(points):
                angle = start_angle + angle_step * i
                angle = angle % 360
                angle_idx = int(angle)
                
                if 0 <= angle_idx < 360 and distance > 0:
                    self.scan_data[angle_idx] = distance
            
            self.scan_ready = True
            
        except Exception as e:
            self.get_logger().debug(f'Parse error: {e}')
    
    def publish_scan(self):
        """发布激光扫描数据"""
        if not self.scan_ready:
            return
        
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # 扫描参数
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = math.pi / 180.0  # 1度
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.02
        msg.range_max = 12.0
        
        # 填充数据
        msg.ranges = self.scan_data.copy()
        msg.intensities = [0.0] * 360
        
        self.scan_pub.publish(msg)
    
    def destroy_node(self):
        """销毁节点"""
        self.running = False
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LDLidarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
