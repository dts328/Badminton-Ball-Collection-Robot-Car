#!/usr/bin/env python3
"""
Behavior Tree Node
行为树节点 - 使用BehaviorTree.CPP风格
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import time


class NodeStatus:
    """节点状态"""
    SUCCESS = 'success'
    FAILURE = 'failure'
    RUNNING = 'running'


class BTNode:
    """行为树节点基类"""
    
    def __init__(self, name):
        self.name = name
        self.children = []
    
    def tick(self):
        """执行一次"""
        raise NotImplementedError
    
    def add_child(self, child):
        """添加子节点"""
        self.children.append(child)


class SequenceNode(BTNode):
    """顺序节点 - 所有子节点都成功才成功"""
    
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status != NodeStatus.SUCCESS:
                return status
        return NodeStatus.SUCCESS


class SelectorNode(BTNode):
    """选择节点 - 任一子节点成功就成功"""
    
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status != NodeStatus.FAILURE:
                return status
        return NodeStatus.FAILURE


class ConditionNode(BTNode):
    """条件节点"""
    
    def __init__(self, name, check_func):
        super().__init__(name)
        self.check_func = check_func
    
    def tick(self):
        if self.check_func():
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class ActionNode(BTNode):
    """动作节点"""
    
    def __init__(self, name, action_func):
        super().__init__(name)
        self.action_func = action_func
    
    def tick(self):
        return self.action_func()


class BehaviorTreeNode(Node):
    """行为树节点"""
    
    def __init__(self):
        super().__init__('behavior_tree')
        
        # 状态变量
        self.target_detected = False
        self.target_confirmed = False
        self.is_patrolling = False
        self.is_grasping = False
        self.task_active = False
        
        # 订阅
        self.detection_sub = self.create_subscription(
            Bool, '/tracking_status', self.detection_callback, 10
        )
        self.task_cmd_sub = self.create_subscription(
            String, '/task/command', self.task_command_callback, 10
        )
        
        # 发布
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/bt/state', 10)
        
        # 构建行为树
        self.root = self.build_behavior_tree()
        
        # 定时器
        self.timer = self.create_timer(0.1, self.tick)  # 10Hz
        
        self.get_logger().info('Behavior Tree started')
    
    def build_behavior_tree(self):
        """构建行为树"""
        # 根节点：任务选择器
        root = SelectorNode("Root")
        
        # 序列1：目标检测与抓取
        grasp_sequence = SequenceNode("GraspSequence")
        grasp_sequence.add_child(ConditionNode("CheckTarget", self.check_target_detected))
        grasp_sequence.add_child(ActionNode("StopPatrol", self.stop_patrol))
        grasp_sequence.add_child(ActionNode("TrackTarget", self.track_target))
        grasp_sequence.add_child(ActionNode("ApproachTarget", self.approach_target))
        grasp_sequence.add_child(ActionNode("GraspTarget", self.grasp_target))
        grasp_sequence.add_child(ActionNode("PlaceTarget", self.place_target))
        
        # 序列2：巡航
        patrol_sequence = SequenceNode("PatrolSequence")
        patrol_sequence.add_child(ConditionNode("CheckActive", self.check_task_active))
        patrol_sequence.add_child(ActionNode("Patrol", self.patrol))
        
        # 组装行为树
        root.add_child(grasp_sequence)
        root.add_child(patrol_sequence)
        
        return root
    
    def tick(self):
        """执行行为树"""
        if self.root:
            status = self.root.tick()
            
            # 发布状态
            state_msg = String()
            state_msg.data = status
            self.state_pub.publish(state_msg)
    
    # ==================== 条件检查 ====================
    
    def check_target_detected(self):
        """检查是否检测到目标"""
        return self.target_confirmed
    
    def check_task_active(self):
        """检查任务是否激活"""
        return self.task_active
    
    # ==================== 动作执行 ====================
    
    def stop_patrol(self):
        """停止巡航"""
        self.is_patrolling = False
        self.get_logger().info('Stopping patrol')
        return NodeStatus.SUCCESS
    
    def track_target(self):
        """追踪目标"""
        self.get_logger().info('Tracking target')
        # 发送追踪速度指令
        return NodeStatus.RUNNING
    
    def approach_target(self):
        """接近目标"""
        self.get_logger().info('Approaching target')
        return NodeStatus.RUNNING
    
    def grasp_target(self):
        """抓取目标"""
        if not self.is_grasping:
            self.is_grasping = True
            self.get_logger().info('Grasping target')
            # 发送夹爪指令
            return NodeStatus.RUNNING
        
        # 等待抓取完成
        time.sleep(1.0)
        self.is_grasping = False
        return NodeStatus.SUCCESS
    
    def place_target(self):
        """放置目标"""
        self.get_logger().info('Placing target')
        # 导航到放置点
        # 打开夹爪
        self.target_confirmed = False
        return NodeStatus.SUCCESS
    
    def patrol(self):
        """巡航"""
        if not self.is_patrolling:
            self.is_patrolling = True
            self.get_logger().info('Starting patrol')
        
        # 执行巡航逻辑
        return NodeStatus.RUNNING
    
    # ==================== 回调函数 ====================
    
    def detection_callback(self, msg):
        """检测状态回调"""
        self.target_confirmed = msg.data
    
    def task_command_callback(self, msg):
        """任务指令回调"""
        cmd = msg.data.strip().upper()
        if cmd == 'START':
            self.task_active = True
        elif cmd == 'STOP':
            self.task_active = False


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
