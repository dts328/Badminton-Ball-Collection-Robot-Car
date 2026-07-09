#!/usr/bin/env python3
"""
Point Cloud Processing Node
点云处理节点 - 深度图转点云
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct


class PointCloudProcessor(Node):
    """点云处理节点"""
    
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        # 参数
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('output_topic', '/camera/depth/points_processed')
        self.declare_parameter('fx', 570.0)
        self.declare_parameter('fy', 570.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('depth_scale', 0.001)  # mm -> m
        
        # 获取参数
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value
        self.depth_scale = self.get_parameter('depth_scale').get_parameter_value().double_value
        
        # 订阅深度图
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )
        
        # 发布点云
        self.cloud_pub = self.create_publisher(PointCloud2, output_topic, 10)
        
        self.get_logger().info('Point Cloud Processor started')
    
    def depth_callback(self, msg):
        """深度图回调"""
        # 解析深度图
        if msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        elif msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            self.get_logger().warn(f'Unsupported encoding: {msg.encoding}')
            return
        
        # 生成点云
        cloud = self.depth_to_pointcloud(depth, msg.header)
        
        # 发布
        self.cloud_pub.publish(cloud)
    
    def depth_to_pointcloud(self, depth, header):
        """深度图转点云"""
        height, width = depth.shape
        
        # 创建像素网格
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        
        # 转换为浮点数
        depth_f = depth.astype(np.float32) * self.depth_scale
        
        # 过滤无效深度
        mask = depth_f > 0
        
        # 计算3D坐标
        z = depth_f[mask]
        x = (u[mask] - self.cx) * z / self.fx
        y = (v[mask] - self.cy) * z / self.fy
        
        # 创建点云消息
        msg = PointCloud2()
        msg.header = header
        
        # 点云字段
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.is_dense = True
        
        # 填充点云数据
        points = np.stack([x, y, z], axis=-1).astype(np.float32)
        num_points = points.shape[0]
        
        msg.width = num_points
        msg.height = 1
        msg.row_step = msg.point_step * num_points
        msg.data = points.tobytes()
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
