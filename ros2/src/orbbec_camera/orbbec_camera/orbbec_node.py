#!/usr/bin/env python3
"""
Orbbec Depth Camera Node
奥比中光深度相机驱动节点
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import time


class OrbbecNode(Node):
    """奥比中光深度相机节点"""
    
    def __init__(self):
        super().__init__('orbbec_camera')
        
        # 参数
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_pointcloud', True)
        self.declare_parameter('frame_id', 'orbbec_frame')
        
        # 获取参数
        self.color_width = self.get_parameter('color_width').get_parameter_value().integer_value
        self.color_height = self.get_parameter('color_height').get_parameter_value().integer_value
        self.depth_width = self.get_parameter('depth_width').get_parameter_value().integer_value
        self.depth_height = self.get_parameter('depth_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # 发布者
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/camera/depth/points', 10)
        
        # 相机参数（需要标定）
        self.color_fx = 616.0
        self.color_fy = 616.0
        self.color_cx = 320.0
        self.color_cy = 240.0
        
        self.depth_fx = 570.0
        self.depth_fy = 570.0
        self.depth_cx = 320.0
        self.depth_cy = 240.0
        
        # 模拟数据（实际使用时替换为SDK调用）
        self.frame_count = 0
        
        # 定时器
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frames)
        
        self.get_logger().info(f'Orbbec Camera started: {self.color_width}x{self.color_height}@{self.fps}fps')
    
    def publish_frames(self):
        """发布相机帧"""
        stamp = self.get_clock().now().to_msg()
        
        # 发布彩色图像
        self.publish_color_image(stamp)
        
        # 发布深度图像
        self.publish_depth_image(stamp)
        
        # 发布点云
        self.publish_pointcloud(stamp)
        
        self.frame_count += 1
    
    def publish_color_image(self, stamp):
        """发布彩色图像"""
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = self.color_height
        msg.width = self.color_width
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = self.color_width * 3
        
        # 生成模拟图像数据
        image = np.zeros((self.color_height, self.color_width, 3), dtype=np.uint8)
        image[:, :, 0] = 128  # B
        image[:, :, 1] = 128  # G
        image[:, :, 2] = 128  # R
        msg.data = image.tobytes()
        
        self.color_pub.publish(msg)
        
        # 发布相机信息
        info = self.create_camera_info(stamp, 'color')
        self.color_info_pub.publish(info)
    
    def publish_depth_image(self, stamp):
        """发布深度图像"""
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = self.depth_height
        msg.width = self.depth_width
        msg.encoding = '16UC1'
        msg.is_bigendian = False
        msg.step = self.depth_width * 2
        
        # 生成模拟深度数据
        depth = np.ones((self.depth_height, self.depth_width), dtype=np.uint16) * 1000  # 1m
        msg.data = depth.tobytes()
        
        self.depth_pub.publish(msg)
        
        # 发布相机信息
        info = self.create_camera_info(stamp, 'depth')
        self.depth_info_pub.publish(info)
    
    def publish_pointcloud(self, stamp):
        """发布点云"""
        # 创建点云消息
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        # 点云字段
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16
        msg.is_dense = True
        
        # 生成模拟点云
        width = 100
        height = 100
        msg.width = width
        msg.height = height
        msg.row_step = msg.point_step * width
        
        points = []
        for v in range(height):
            for u in range(width):
                x = (u - width/2) * 0.01
                y = (v - height/2) * 0.01
                z = 1.0
                rgb = struct.unpack('f', struct.pack('BBBB', 128, 128, 128, 255))[0]
                points.extend([x, y, z, rgb])
        
        msg.data = np.array(points, dtype=np.float32).tobytes()
        
        self.cloud_pub.publish(msg)
    
    def create_camera_info(self, stamp, camera_type):
        """创建相机信息"""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        
        if camera_type == 'color':
            info.width = self.color_width
            info.height = self.color_height
            info.k = [self.color_fx, 0, self.color_cx,
                      0, self.color_fy, self.color_cy,
                      0, 0, 1]
            info.p = [self.color_fx, 0, self.color_cx, 0,
                      0, self.color_fy, self.color_cy, 0,
                      0, 0, 1, 0]
        else:
            info.width = self.depth_width
            info.height = self.depth_height
            info.k = [self.depth_fx, 0, self.depth_cx,
                      0, self.depth_fy, self.depth_cy,
                      0, 0, 1]
            info.p = [self.depth_fx, 0, self.depth_cx, 0,
                      0, self.depth_fy, self.depth_cy, 0,
                      0, 0, 1, 0]
        
        return info


def main(args=None):
    rclpy.init(args=args)
    node = OrbbecNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
