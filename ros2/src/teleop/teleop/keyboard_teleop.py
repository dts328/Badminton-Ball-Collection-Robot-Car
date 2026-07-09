#!/usr/bin/env python3
"""
Keyboard Teleop Node
键盘遥控节点 - WASD控制底盘，IK控制机械臂
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import sys
import termios
import tty
import threading


class KeyboardTeleop(Node):
    """键盘遥控节点"""
    
    # 速度限制
    MAX_LINEAR_VEL = 0.5   # m/s
    MAX_ANGULAR_VEL = 1.0  # rad/s
    
    # 速度步长
    LINEAR_STEP = 0.05
    ANGULAR_STEP = 0.1
    
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper/close', 10)
        self.task_cmd_pub = self.create_publisher(String, '/task/command', 10)
        
        # 当前速度
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 启动输入线程
        self.running = True
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
        # 定时发布
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10Hz
        
        self.print_instructions()
    
    def print_instructions(self):
        """打印控制说明"""
        msg = """
==========================================
  Smart Robot Keyboard Teleop
==========================================

  移动控制:
    w/x : 前进/后退
    a/d : 左移/右移
    q/e : 左转/右转
    s   : 停止

  速度调节:
    +/- : 增加/减少线速度
    [/] : 增加/减少角速度

  夹爪控制:
    o   : 打开夹爪
    c   : 关闭夹爪

  任务控制:
    1   : 开始任务
    2   : 停止任务
    3   : 开始巡航
    4   : 回到起点

  其他:
    space : 紧急停止
    q     : 退出

  当前速度: linear=%.2f m/s, angular=%.2f rad/s
==========================================
"""
        self.get_logger().info(msg % (self.MAX_LINEAR_VEL, self.MAX_ANGULAR_VEL))
    
    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def input_loop(self):
        """输入循环"""
        while self.running:
            try:
                key = self.get_key()
                self.process_key(key)
            except Exception as e:
                self.get_logger().error(f'Input error: {e}')
                break
    
    def process_key(self, key):
        """处理按键"""
        # 移动控制
        if key == 'w':
            self.linear_x = min(self.linear_x + self.LINEAR_STEP, self.MAX_LINEAR_VEL)
        elif key == 'x':
            self.linear_x = max(self.linear_x - self.LINEAR_STEP, -self.MAX_LINEAR_VEL)
        elif key == 'a':
            self.linear_y = min(self.linear_y + self.LINEAR_STEP, self.MAX_LINEAR_VEL)
        elif key == 'd':
            self.linear_y = max(self.linear_y - self.LINEAR_STEP, -self.MAX_LINEAR_VEL)
        elif key == 'q':
            self.angular_z = min(self.angular_z + self.ANGULAR_STEP, self.MAX_ANGULAR_VEL)
        elif key == 'e':
            self.angular_z = max(self.angular_z - self.ANGULAR_STEP, -self.MAX_ANGULAR_VEL)
        elif key == 's':
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0
        
        # 速度调节
        elif key == '+':
            self.MAX_LINEAR_VEL = min(self.MAX_LINEAR_VEL + 0.1, 1.0)
        elif key == '-':
            self.MAX_LINEAR_VEL = max(self.MAX_LINEAR_VEL - 0.1, 0.1)
        elif key == '[':
            self.MAX_ANGULAR_VEL = min(self.MAX_ANGULAR_VEL + 0.1, 2.0)
        elif key == ']':
            self.MAX_ANGULAR_VEL = max(self.MAX_ANGULAR_VEL - 0.1, 0.1)
        
        # 夹爪控制
        elif key == 'o':
            self.control_gripper(True)
        elif key == 'c':
            self.control_gripper(False)
        
        # 任务控制
        elif key == '1':
            self.send_task_command('START')
        elif key == '2':
            self.send_task_command('STOP')
        elif key == '3':
            self.send_task_command('PATROL')
        elif key == '4':
            self.send_task_command('HOME')
        
        # 紧急停止
        elif key == ' ':
            self.emergency_stop()
        
        # 退出
        elif key == '\x03' or key == 'q':  # Ctrl+C or q
            self.running = False
            return
        
        # 打印状态
        self.print_status()
    
    def publish_velocity(self):
        """发布速度指令"""
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y
        msg.angular.z = self.angular_z
        self.cmd_vel_pub.publish(msg)
    
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
    
    def emergency_stop(self):
        """紧急停止"""
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.publish_velocity()
        self.get_logger().warn('紧急停止!')
    
    def print_status(self):
        """打印状态"""
        self.get_logger().info(
            f'速度: vx={self.linear_x:.2f} vy={self.linear_y:.2f} '
            f'w={self.angular_z:.2f} | '
            f'限速: {self.MAX_LINEAR_VEL:.1f}/{self.MAX_ANGULAR_VEL:.1f}'
        )
    
    def destroy_node(self):
        """销毁节点"""
        self.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
