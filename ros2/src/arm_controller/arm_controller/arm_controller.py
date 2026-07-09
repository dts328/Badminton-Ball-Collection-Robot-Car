#!/usr/bin/env python3
"""
Arm Controller Node
机械臂控制节点 - 包含关节控制和夹爪控制

STM32支持的命令格式:
- $KMS:x,y,z,time!  - 笛卡尔坐标控制 (逆运动学)
- #IDPpwmTtime!     - 舵机控制 (ID=舵机号, PWM=脉宽, time=时间)
- $DST!             - 舵机停止
- $DJR!             - 舵机复位

舵机分配:
- Servo 0: Base (底座)
- Servo 1: Shoulder (肩)
- Servo 2: Elbow (肘)
- Servo 3: Wrist (腕)
- Servo 4: Gripper (夹爪)
- Servo 5: Spare (备用)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import serial
import time


class ArmController(Node):
    """机械臂控制节点"""
    
    # 舵机ID
    SERVO_BASE = 0
    SERVO_SHOULDER = 1
    SERVO_ELBOW = 2
    SERVO_WRIST = 3
    SERVO_GRIPPER = 4
    
    # 夹爪PWM值
    GRIPPER_OPEN = 2000    # 打开位置
    GRIPPER_CLOSE = 1000   # 关闭位置
    GRIPPER_CENTER = 1500  # 中间位置
    
    # 舵机PWM范围
    SERVO_PWM_MIN = 500
    SERVO_PWM_MAX = 2500
    SERVO_PWM_CENTER = 1500
    
    def __init__(self):
        super().__init__('arm_controller')
        
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
        self.preset_sub = self.create_subscription(
            String, '/arm/preset', self.preset_callback, 10
        )
        self.servo_sub = self.create_subscription(
            String, '/arm/servo', self.servo_callback, 10
        )
        self.gripper_open_sub = self.create_subscription(
            Bool, '/gripper/open', self.gripper_open_callback, 10
        )
        self.gripper_close_sub = self.create_subscription(
            Bool, '/gripper/close', self.gripper_close_callback, 10
        )
        
        # 发布
        self.state_pub = self.create_publisher(String, '/arm/state', 10)
        self.gripper_state_pub = self.create_publisher(Bool, '/gripper/state', 10)
        
        # 当前状态
        self.is_gripper_open = True
        self.current_pwm = [SERVO_PWM_CENTER] * 6
        
        self.get_logger().info(f'Arm Controller started: {port_name}@{baudrate}')
        self.get_logger().info('Servo mapping: 0=Base, 1=Shoulder, 2=Elbow, 3=Wrist, 4=Gripper')
    
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
    
    def preset_callback(self, msg):
        """预设姿态回调
        格式: home/ready/grasp/place/stop/reset
        """
        cmd = msg.data.strip().upper()
        
        if cmd == 'HOME':
            # 归零位置
            self.send_servo_command(self.SERVO_BASE, self.SERVO_PWM_CENTER, 1000)
            self.send_servo_command(self.SERVO_SHOULDER, self.SERVO_PWM_CENTER, 1000)
            self.send_servo_command(self.SERVO_ELBOW, self.SERVO_PWM_CENTER, 1000)
            self.send_servo_command(self.SERVO_WRIST, self.SERVO_PWM_CENTER, 1000)
            self.send_servo_command(self.SERVO_GRIPPER, self.GRIPPER_OPEN, 500)
            
        elif cmd == 'READY':
            # 准备姿态
            self.send_servo_command(self.SERVO_BASE, 1500, 500)
            self.send_servo_command(self.SERVO_SHOULDER, 1000, 500)
            self.send_servo_command(self.SERVO_ELBOW, 2000, 500)
            self.send_servo_command(self.SERVO_WRIST, 1500, 500)
            self.send_servo_command(self.SERVO_GRIPPER, self.GRIPPER_OPEN, 500)
            
        elif cmd == 'GRASP':
            # 抓取姿态
            self.send_servo_command(self.SERVO_BASE, 1500, 500)
            self.send_servo_command(self.SERVO_SHOULDER, 1200, 500)
            self.send_servo_command(self.SERVO_ELBOW, 1800, 500)
            self.send_servo_command(self.SERVO_WRIST, 1200, 500)
            
        elif cmd == 'PLACE':
            # 放置姿态
            self.send_servo_command(self.SERVO_BASE, 2000, 500)
            self.send_servo_command(self.SERVO_SHOULDER, 1200, 500)
            self.send_servo_command(self.SERVO_ELBOW, 1800, 500)
            self.send_servo_command(self.SERVO_WRIST, 1200, 500)
            
        elif cmd == 'STOP':
            # 停止所有舵机
            self.send_stop_command()
            
        elif cmd == 'RESET':
            # 复位所有舵机
            self.send_reset_command()
            
        else:
            self.get_logger().warn(f'Unknown preset: {cmd}')
            return
        
        self.get_logger.info(f'Preset executed: {cmd}')
    
    def servo_callback(self, msg):
        """单舵机控制回调
        格式: servo_id,pwm,time_ms
        示例: 4,2000,500  (舵机4移动到2000us，耗时500ms)
        """
        try:
            parts = msg.data.split(',')
            if len(parts) >= 2:
                servo_id = int(parts[0])
                pwm = int(parts[1])
                time_ms = int(parts[2]) if len(parts) > 2 else 500
                self.send_servo_command(servo_id, pwm, time_ms)
        except Exception as e:
            self.get_logger().error(f'Servo command error: {e}')
    
    def gripper_open_callback(self, msg):
        """夹爪打开回调"""
        if msg.data:
            self.open_gripper()
    
    def gripper_close_callback(self, msg):
        """夹爪关闭回调"""
        if msg.data:
            self.close_gripper()
    
    def send_servo_command(self, servo_id, pwm, time_ms=500):
        """发送舵机控制命令
        
        格式: #IDPpwmTtime!
        示例: #004P2000T500!
        """
        if self.serial_port is None:
            return False
        
        # 限幅
        pwm = max(self.SERVO_PWM_MIN, min(self.SERVO_PWM_MAX, pwm))
        time_ms = min(9999, time_ms)
        
        # 构造命令
        cmd = f'#{servo_id:03d}P{pwm}T{time_ms}!\n'
        
        try:
            self.serial_port.write(cmd.encode())
            self.current_pwm[servo_id] = pwm
            self.get_logger().info(f'Servo cmd: {cmd.strip()}')
            
            # 发布状态
            state_msg = String()
            state_msg.data = f'SERVO:{servo_id},{pwm},{time_ms}'
            self.state_pub.publish(state_msg)
            
            return True
        except Exception as e:
            self.get_logger().error(f'Servo send failed: {e}')
            self.connect_serial()
            return False
    
    def send_all_servos(self, pwm_list, time_ms=500):
        """同时控制所有舵机
        格式: #255PpwmTtime!
        """
        if self.serial_port is None:
            return False
        
        # 使用255控制所有舵机
        pwm = self.SERVO_PWM_CENTER  # 使用中间值
        cmd = f'#255P{pwm}T{time_ms}!\n'
        
        try:
            self.serial_port.write(cmd.encode())
            self.get_logger().info(f'All servos cmd: {cmd.strip()}')
            return True
        except Exception as e:
            self.get_logger().error(f'All servos send failed: {e}')
            return False
    
    def open_gripper(self):
        """打开夹爪"""
        if self.send_servo_command(self.SERVO_GRIPPER, self.GRIPPER_OPEN, 500):
            self.is_gripper_open = True
            self.publish_gripper_state()
            self.get_logger().info('Gripper opened')
    
    def close_gripper(self):
        """关闭夹爪"""
        if self.send_servo_command(self.SERVO_GRIPPER, self.GRIPPER_CLOSE, 500):
            self.is_gripper_open = False
            self.publish_gripper_state()
            self.get_logger().info('Gripper closed')
    
    def send_stop_command(self):
        """停止所有舵机
        格式: #xxxPDST!
        """
        if self.serial_port is None:
            return False
        
        try:
            # 停止舵机4（夹爪）
            cmd = '#004PDST!\n'
            self.serial_port.write(cmd.encode())
            self.get_logger().info('Servo stop command sent')
            return True
        except Exception as e:
            self.get_logger().error(f'Servo stop failed: {e}')
            return False
    
    def send_reset_command(self):
        """复位所有舵机到中心位置"""
        self.send_all_servos(self.SERVO_PWM_CENTER, 1000)
        self.open_gripper()
    
    def publish_gripper_state(self):
        """发布夹爪状态"""
        msg = Bool()
        msg.data = self.is_gripper_open
        self.gripper_state_pub.publish(msg)
    
    def stop(self):
        """停止所有舵机"""
        self.send_stop_command()
    
    def destroy_node(self):
        """销毁节点"""
        self.stop()
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
