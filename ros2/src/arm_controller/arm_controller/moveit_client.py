#!/usr/bin/env python3
"""
MoveIt Client Node
机械臂MoveIt2接口节点
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
import serial
import threading
import time


class MoveItClient(Node):
    """MoveIt2客户端节点"""
    
    def __init__(self):
        super().__init__('arm_controller')
        
        # 参数
        self.declare_parameter('port_name', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'])
        
        port_name = self.get_parameter('port_name').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        
        # 串口连接
        self.serial_port = None
        self.port_name = port_name
        self.baudrate = baudrate
        self.connect_serial()
        
        # Action客户端
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 订阅关节指令
        self.joint_sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.joint_callback,
            10
        )
        
        # 当前关节角度
        self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info(f'Arm Controller started: {port_name}@{baudrate}')
    
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.get_logger().info(f'Arm serial connected: {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Arm serial connect failed: {e}')
            self.serial_port = None
    
    def joint_callback(self, msg):
        """关节指令回调"""
        if len(msg.points) == 0:
            return
        
        # 获取目标角度
        point = msg.points[0]
        angles = point.positions
        
        # 发送到STM32
        self.send_joint_angles(angles)
    
    def send_joint_angles(self, angles):
        """发送关节角度到STM32
        
        格式: $ARM:j1,j2,j3,j4,j5,grip!
        """
        if self.serial_port is None:
            return False
        
        # 构造指令
        angle_str = ','.join([f'{a:.3f}' for a in angles[:5]])
        cmd = f'$ARM:{angle_str},0.0!\n'
        
        try:
            self.serial_port.write(cmd.encode())
            self.current_angles = list(angles[:5])
            self.get_logger().debug(f'Sent arm cmd: {cmd.strip()}')
            return True
        except Exception as e:
            self.get_logger().error(f'Arm send failed: {e}')
            self.connect_serial()
            return False
    
    def send_home(self):
        """回到归零位置"""
        return self.send_joint_angles([0.0, 0.0, 0.0, 0.0, 0.0])
    
    def send_ready(self):
        """移动到准备位置"""
        return self.send_joint_angles([0.0, -1.57, 1.57, 0.0, 0.0])
    
    def send_grasp(self):
        """移动到抓取位置"""
        return self.send_joint_angles([0.0, -0.785, 1.57, -0.785, 0.0])
    
    def send_place(self):
        """移动到放置位置"""
        return self.send_joint_angles([1.57, -0.785, 1.57, -0.785, 0.0])
    
    def stop(self):
        """停止运动"""
        if self.serial_port:
            try:
                self.serial_port.write(b'$DST!\n')
                return True
            except:
                pass
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MoveItClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
