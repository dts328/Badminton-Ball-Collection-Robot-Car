#!/usr/bin/env python3
"""
Gripper Control Node
夹爪控制节点
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import serial
import time


class GripperControl(Node):
    """夹爪控制节点"""
    
    # 夹爪状态
    OPEN = 0.025   # 打开位置(m)
    CLOSE = 0.0    # 关闭位置(m)
    
    def __init__(self):
        super().__init__('gripper_control')
        
        # 参数
        self.declare_parameter('port_name', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        
        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        
        # 串口
        self.serial_port = None
        self.port_name = port_name
        self.baudrate = baudrate
        self.connect_serial()
        
        # 订阅
        self.open_sub = self.create_subscription(
            Bool, '/gripper/open', self.open_callback, 10
        )
        self.close_sub = self.create_subscription(
            Bool, '/gripper/close', self.close_callback, 10
        )
        self.position_sub = self.create_subscription(
            Float32, '/gripper/position', self.position_callback, 10
        )
        
        # 发布状态
        self.state_pub = self.create_publisher(Bool, '/gripper/state', 10)
        
        # 当前状态
        self.is_open = True
        
        self.get_logger().info(f'Gripper Control started: {port_name}@{baudrate}')
    
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=0.1
            )
        except Exception as e:
            self.get_logger().error(f'Gripper serial connect failed: {e}')
            self.serial_port = None
    
    def open_callback(self, msg):
        """打开夹爪"""
        if msg.data:
            self.open_gripper()
    
    def close_callback(self, msg):
        """关闭夹爪"""
        if msg.data:
            self.close_gripper()
    
    def position_callback(self, msg):
        """设置夹爪位置"""
        self.set_position(msg.data)
    
    def open_gripper(self):
        """打开夹爪"""
        if self.send_command('$GRIP:OPEN!'):
            self.is_open = True
            self.publish_state()
            self.get_logger().info('Gripper opened')
    
    def close_gripper(self):
        """关闭夹爪"""
        if self.send_command('$GRIP:CLOSE!'):
            self.is_open = False
            self.publish_state()
            self.get_logger().info('Gripper closed')
    
    def set_position(self, position):
        """设置夹爪位置"""
        cmd = f'$GRIP:{position:.3f}!'
        if self.send_command(cmd):
            self.is_open = position > self.OPEN / 2
            self.publish_state()
    
    def send_command(self, cmd):
        """发送命令"""
        if self.serial_port is None:
            return False
        
        try:
            self.serial_port.write(f'{cmd}\n'.encode())
            return True
        except Exception as e:
            self.get_logger().error(f'Gripper send failed: {e}')
            self.connect_serial()
            return False
    
    def publish_state(self):
        """发布状态"""
        msg = Bool()
        msg.data = self.is_open
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
