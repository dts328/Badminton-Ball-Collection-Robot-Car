#!/usr/bin/env python3
"""
Odometry Publisher Node
里程计发布节点 - 麦克纳姆轮正运动学
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_msgs.msg import UInt8MultiArray
from tf2_ros import TransformBroadcaster
import struct
import math


def parse_status_frame(buf):
    """解析状态帧 (26字节)"""
    if len(buf) != 26:
        return None
    
    if buf[0] != 0x7B or buf[25] != 0x7D:
        return None
    
    # 验证校验
    checksum = 0
    for i in range(24):
        checksum ^= buf[i]
    if buf[24] != checksum:
        return None
    
    # 解析编码器速度 (mm/s -> m/s)
    enc_a = struct.unpack('>h', buf[2:4])[0] / 1000.0
    enc_b = struct.unpack('>h', buf[4:6])[0] / 1000.0
    enc_c = struct.unpack('>h', buf[6:8])[0] / 1000.0
    enc_d = struct.unpack('>h', buf[8:10])[0] / 1000.0
    
    # 电池电压
    battery = struct.unpack('>H', buf[22:24])[0] / 1000.0
    
    return {
        'encoders': [enc_a, enc_b, enc_c, enc_d],
        'battery': battery
    }


class OdometryPublisher(Node):
    """里程计发布节点"""
    
    def __init__(self):
        super().__init__('odom_publisher')
        
        # 麦克纳姆轮参数
        self.declare_parameter('wheel_radius', 0.0485)
        self.declare_parameter('wheelbase', 0.12)
        self.declare_parameter('wheeltrack', 0.236)
        
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.wheeltrack = self.get_parameter('wheeltrack').get_parameter_value().double_value
        
        # 里程计发布
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF广播
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅底盘状态 (二进制)
        self.status_sub = self.create_subscription(
            UInt8MultiArray,
            '/chassis_status',
            self.status_callback,
            10
        )
        
        # 位姿状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        
        # 上次时间
        self.last_time = self.get_clock().now()
        
        # 定时发布
        self.timer = self.create_timer(0.02, self.publish_odom)  # 50Hz
        
        self.get_logger().info('Odometry Publisher started')
    
    def status_callback(self, msg):
        """状态回调 - 解析二进制帧"""
        status = parse_status_frame(msg.data)
        
        if status is not None:
            # 四轮速度 (m/s)
            enc = status['encoders']
            
            # 麦克纳姆轮正运动学
            # vx = (enc_a + enc_b + enc_c + enc_d) / 4
            # vy = (-enc_a + enc_b + enc_c - enc_d) / 4
            # vw = (-enc_a + enc_b - enc_c + enc_d) / (4 * (Lx + Ly))
            
            Lx = self.wheelbase / 2.0  # 前后距离的一半
            Ly = self.wheeltrack / 2.0  # 左右距离的一半
            L = Lx + Ly
            
            self.vx = (enc[0] + enc[1] + enc[2] + enc[3]) / 4.0
            self.vy = (-enc[0] + enc[1] + enc[2] - enc[3]) / 4.0
            self.vw = (-enc[0] + enc[1] - enc[2] + enc[3]) / (4.0 * L)
            
            self.get_logger().debug(
                f'Encoders: [{enc[0]:.3f}, {enc[1]:.3f}, {enc[2]:.3f}, {enc[3]:.3f}]'
            )
    
    def publish_odom(self):
        """发布里程计"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # 积分计算位姿
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_theta = self.vw * dt
        
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # 规范化角度
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # 构造四元数
        q = self.euler_to_quaternion(0, 0, self.theta)
        
        # 发布里程计
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vw
        
        self.odom_pub.publish(odom)
        
        # 发布TF
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(transform)
        
        self.last_time = current_time
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数"""
        q = Quaternion()
        q.w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
              math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        q.x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
              math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        q.y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
              math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        q.z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
              math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
