#!/usr/bin/env python3
"""
Depth Converter Node
2D像素坐标 + 深度图 -> 3D空间坐标
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import numpy as np
import threading


class DepthConverter(Node):
    """深度坐标转换节点"""
    
    def __init__(self):
        super().__init__('depth_converter')
        
        # 参数
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('detection_topic', '/detection/pixel')
        self.declare_parameter('output_topic', '/target_3d_pose')
        self.declare_parameter('fx', 570.0)
        self.declare_parameter('fy', 570.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        
        # 获取参数
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value
        
        # 深度图缓存
        self.depth_image = None
        self.depth_lock = threading.Lock()
        
        # 订阅
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10
        )
        self.detection_sub = self.create_subscription(
            PoseStamped, detection_topic, self.detection_callback, 10
        )
        
        # 发布
        self.pose_pub = self.create_publisher(PoseStamped, output_topic, 10)
        
        self.get_logger().info('Depth Converter started')
    
    def depth_callback(self, msg):
        """深度图回调"""
        with self.depth_lock:
            if msg.encoding == '16UC1':
                self.depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            elif msg.encoding == '32FC1':
                self.depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
    
    def detection_callback(self, msg):
        """检测结果回调 - 像素坐标"""
        with self.depth_lock:
            if self.depth_image is None:
                return
            
            # 获取像素坐标（假设msg中的x,y是像素坐标）
            u = int(msg.pose.position.x)
            v = int(msg.pose.position.y)
            
            # 检查边界
            h, w = self.depth_image.shape
            if u < 0 or u >= w or v < 0 or v >= h:
                return
            
            # 获取深度值
            depth = self.depth_image[v, u]
            
            if depth == 0:
                return
            
            # 深度值转换（mm -> m）
            if self.depth_image.dtype == np.uint16:
                z = depth * 0.001
            else:
                z = float(depth)
            
            # 计算3D坐标
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy
            
            # 发布3D位姿
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'orbbec_frame'
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.w = 1.0
            
            self.pose_pub.publish(pose_msg)
            
            self.get_logger().debug(
                f'Pixel ({u},{v}) -> 3D ({x:.3f}, {y:.3f}, {z:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DepthConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
