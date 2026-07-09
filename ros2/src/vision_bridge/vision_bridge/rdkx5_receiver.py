#!/usr/bin/env python3
"""
RDK X5 Receiver Node
接收RDK X5视觉识别结果
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Int32
import serial
import threading
import json
import time


class RDKX5Receiver(Node):
    """RDK X5视觉接收节点"""
    
    def __init__(self):
        super().__init__('rdkx5_receiver')
        
        # 参数
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'orbbec_frame')
        
        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # 发布者 - 发布到target_3d_pose与depth_converter输出一致
        self.detection_pub = self.create_publisher(PoseStamped, '/target_3d_pose', 10)
        self.class_pub = self.create_publisher(Int32, '/detection/class', 10)
        self.raw_pub = self.create_publisher(String, '/detection/raw', 10)
        
        # 串口
        self.serial_port = None
        self.port_name = port_name
        self.baudrate = baudrate
        self.connect_serial()
        
        # 接收线程
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        self.get_logger().info(f'RDK X5 Receiver started: {port_name}@{baudrate}')
    
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
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode().strip()
                    if line:
                        self.parse_detection(line)
            except Exception as e:
                self.get_logger().debug(f'Receive error: {e}')
                time.sleep(0.01)
    
    def parse_detection(self, line):
        """解析识别结果
        
        预期格式: DET:x,y,z,class_id,confidence
        x,y,z: 目标在相机坐标系下的坐标(m)
        class_id: 0=完好球, 1=破损球
        confidence: 置信度
        """
        try:
            # 发布原始数据
            raw_msg = String()
            raw_msg.data = line
            self.raw_pub.publish(raw_msg)
            
            if not line.startswith('DET:'):
                return
            
            # 解析数据
            data = line[4:].split(',')
            if len(data) >= 5:
                x = float(data[0])
                y = float(data[1])
                z = float(data[2])
                class_id = int(data[3])
                confidence = float(data[4])
                
                # 发布检测结果
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = self.frame_id
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z
                pose_msg.pose.orientation.w = 1.0
                self.detection_pub.publish(pose_msg)
                
                # 发布类别
                class_msg = Int32()
                class_msg.data = class_id
                self.class_pub.publish(class_msg)
                
                self.get_logger().info(
                    f'Detected: class={class_id}, '
                    f'pos=({x:.3f}, {y:.3f}, {z:.3f}), '
                    f'conf={confidence:.3f}'
                )
                
        except Exception as e:
            self.get_logger().debug(f'Parse error: {e}')
    
    def destroy_node(self):
        """销毁节点"""
        self.running = False
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RDKX5Receiver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
