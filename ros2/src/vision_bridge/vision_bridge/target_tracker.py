#!/usr/bin/env python3
"""
Target Tracker Node
目标追踪器 - 15帧滑窗投票机制
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool
from collections import deque
import threading


class TargetTracker(Node):
    """目标追踪节点 - 15帧滑窗投票"""
    
    def __init__(self):
        super().__init__('target_tracker')
        
        # 参数
        self.declare_parameter('window_size', 15)
        self.declare_parameter('vote_threshold', 8)
        self.declare_parameter('input_topic', '/target_3d_pose')
        self.declare_parameter('class_topic', '/detection/class')
        self.declare_parameter('output_topic', '/target_confirmed')
        self.declare_parameter('status_topic', '/tracking_status')
        
        # 获取参数
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        self.vote_threshold = self.get_parameter('vote_threshold').get_parameter_value().integer_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        class_topic = self.get_parameter('class_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        
        # 滑动窗口
        self.detection_queue = deque(maxlen=self.window_size)
        self.class_queue = deque(maxlen=self.window_size)
        self.lock = threading.Lock()
        
        # 订阅
        self.pose_sub = self.create_subscription(
            PoseStamped, input_topic, self.pose_callback, 10
        )
        self.class_sub = self.create_subscription(
            Int32, class_topic, self.class_callback, 10
        )
        
        # 发布
        self.confirmed_pub = self.create_publisher(PoseStamped, output_topic, 10)
        self.status_pub = self.create_publisher(Bool, status_topic, 10)
        
        # 统计
        self.total_frames = 0
        self.confirmed_frames = 0
        
        self.get_logger().info(
            f'Target Tracker started: window={self.window_size}, threshold={self.vote_threshold}'
        )
    
    def class_callback(self, msg):
        """类别回调"""
        with self.lock:
            self.class_queue.append(msg.data)
    
    def pose_callback(self, msg):
        """位姿回调"""
        with self.lock:
            self.total_frames += 1
            
            # 添加到队列
            self.detection_queue.append(msg)
            
            # 检查是否达到投票阈值
            if len(self.detection_queue) >= self.window_size:
                # 统计有效检测
                valid_count = sum(1 for p in self.detection_queue if p is not None)
                
                # 投票判定
                if valid_count >= self.vote_threshold:
                    self.confirmed_frames += 1
                    
                    # 计算平均位姿
                    avg_pose = self.calculate_average_pose()
                    
                    # 发布确认结果
                    self.confirmed_pub.publish(avg_pose)
                    
                    # 发布状态
                    status_msg = Bool()
                    status_msg.data = True
                    self.status_pub.publish(status_msg)
                    
                    self.get_logger().info(
                        f'Target confirmed: {valid_count}/{self.window_size} votes, '
                        f'pos=({avg_pose.pose.position.x:.3f}, '
                        f'{avg_pose.pose.position.y:.3f}, '
                        f'{avg_pose.pose.position.z:.3f})'
                    )
                else:
                    # 目标未确认
                    status_msg = Bool()
                    status_msg.data = False
                    self.status_pub.publish(status_msg)
                    
                    self.get_logger().debug(
                        f'Target not confirmed: {valid_count}/{self.window_size} votes'
                    )
    
    def calculate_average_pose(self):
        """计算平均位姿"""
        # 过滤有效检测
        valid_poses = [p for p in self.detection_queue if p is not None]
        
        if not valid_poses:
            return PoseStamped()
        
        # 计算平均值
        avg_x = sum(p.pose.position.x for p in valid_poses) / len(valid_poses)
        avg_y = sum(p.pose.position.y for p in valid_poses) / len(valid_poses)
        avg_z = sum(p.pose.position.z for p in valid_poses) / len(valid_poses)
        
        # 构造消息
        avg_pose = PoseStamped()
        avg_pose.header.stamp = self.get_clock().now().to_msg()
        avg_pose.header.frame_id = valid_poses[0].header.frame_id
        avg_pose.pose.position.x = avg_x
        avg_pose.pose.position.y = avg_y
        avg_pose.pose.position.z = avg_z
        avg_pose.pose.orientation.w = 1.0
        
        return avg_pose
    
    def get_statistics(self):
        """获取统计信息"""
        if self.total_frames == 0:
            return 0.0
        return self.confirmed_frames / self.total_frames


def main(args=None):
    rclpy.init(args=args)
    node = TargetTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stats = node.get_statistics()
        node.get_logger().info(f'Tracking statistics: {stats:.2%} confirmation rate')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
