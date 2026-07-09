#!/usr/bin/env python3
"""
Joy Teleop Node
手柄遥控节点 - 游戏手柄控制
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


class JoyTeleop(Node):
    """手柄遥控节点"""
    
    def __init__(self):
        super().__init__('joy_teleop')
        
        # 参数
        self.declare_parameter('axis_linear_x', 1)      # 左摇杆Y轴
        self.declare_parameter('axis_linear_y', 0)      # 左摇杆X轴
        self.declare_parameter('axis_angular_z', 3)     # 右摇杆X轴
        self.declare_parameter('axis_deadman', 4)       # LT扳机
        self.declare_parameter('button_gripper_open', 0)   # A按钮
        self.declare_parameter('button_gripper_close', 1)  # B按钮
        self.declare_parameter('button_task_start', 3)     # Y按钮
        self.declare_parameter('button_task_stop', 2)      # X按钮
        self.declare_parameter('scale_linear', 0.5)
        self.declare_parameter('scale_angular', 1.0)
        
        # 获取参数
        self.axis_linear_x = self.get_parameter('axis_linear_x').get_parameter_value().integer_value
        self.axis_linear_y = self.get_parameter('axis_linear_y').get_parameter_value().integer_value
        self.axis_angular_z = self.get_parameter('axis_angular_z').get_parameter_value().integer_value
        self.axis_deadman = self.get_parameter('axis_deadman').get_parameter_value().integer_value
        self.button_gripper_open = self.get_parameter('button_gripper_open').get_parameter_value().integer_value
        self.button_gripper_close = self.get_parameter('button_gripper_close').get_parameter_value().integer_value
        self.button_task_start = self.get_parameter('button_task_start').get_parameter_value().integer_value
        self.button_task_stop = self.get_parameter('button_task_stop').get_parameter_value().integer_value
        self.scale_linear = self.get_parameter('scale_linear').get_parameter_value().double_value
        self.scale_angular = self.get_parameter('scale_angular').get_parameter_value().double_value
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper/close', 10)
        self.task_cmd_pub = self.create_publisher(String, '/task/command', 10)
        
        # 订阅手柄
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 上一次按钮状态
        self.last_buttons = [0] * 16
        
        self.get_logger().info('Joy Teleop started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Stick: Move X/Y')
        self.get_logger().info('  Right Stick: Rotate')
        self.get_logger().info('  LT: Deadman switch')
        self.get_logger().info('  A: Open gripper')
        self.get_logger().info('  B: Close gripper')
        self.get_logger().info('  Y: Start task')
        self.get_logger().info('  X: Stop task')
    
    def joy_callback(self, msg):
        """手柄回调"""
        # 检查死人开关
        if len(msg.axes) > self.axis_deadman:
            deadman = msg.axes[self.axis_deadman]
            if deadman < 0.5:  # LT未按下
                self.stop()
                return
        
        # 速度指令
        cmd = Twist()
        
        if len(msg.axes) > max(self.axis_linear_x, self.axis_linear_y, self.axis_angular_z):
            cmd.linear.x = msg.axes[self.axis_linear_x] * self.scale_linear
            cmd.linear.y = msg.axes[self.axis_linear_y] * self.scale_linear
            cmd.angular.z = msg.axes[self.axis_angular_z] * self.scale_angular
        
        self.cmd_vel_pub.publish(cmd)
        
        # 按钮处理
        if len(msg.buttons) > max(self.button_gripper_open, self.button_gripper_close, 
                                   self.button_task_start, self.button_task_stop):
            # 夹爪打开 (A按钮按下)
            if msg.buttons[self.button_gripper_open] and not self.last_buttons[self.button_gripper_open]:
                self.control_gripper(True)
            
            # 夹爪关闭 (B按钮按下)
            if msg.buttons[self.button_gripper_close] and not self.last_buttons[self.button_gripper_close]:
                self.control_gripper(False)
            
            # 开始任务 (Y按钮按下)
            if msg.buttons[self.button_task_start] and not self.last_buttons[self.button_task_start]:
                self.send_task_command('START')
            
            # 停止任务 (X按钮按下)
            if msg.buttons[self.button_task_stop] and not self.last_buttons[self.button_task_stop]:
                self.send_task_command('STOP')
            
            # 保存按钮状态
            self.last_buttons = list(msg.buttons)
    
    def stop(self):
        """停止"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def control_gripper(self, open_gripper):
        """控制夹爪"""
        msg = Bool()
        msg.data = open_gripper
        self.gripper_pub.publish(msg)
        action = "打开" if open_gripper else "关闭"
        self.get_logger().info(f'夹爪{action}')
    
    def send_task_command(self, cmd):
        """发送任务指令"""
        msg = String()
        msg.data = cmd
        self.task_cmd_pub.publish(msg)
        self.get_logger().info(f'任务指令: {cmd}')


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
