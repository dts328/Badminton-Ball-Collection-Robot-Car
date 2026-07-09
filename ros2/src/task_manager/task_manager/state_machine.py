#!/usr/bin/env python3
"""
State Machine Node
任务状态机节点 - 管理巡航/追踪/抓取/放置
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String, Int32
from nav2_msgs.action import NavigateToPose
import math
import time


class TaskState:
    """任务状态枚举"""
    IDLE = 'idle'
    PATROL = 'patrol'
    DETECTED = 'detected'
    TRACKING = 'tracking'
    APPROACHING = 'approaching'
    GRASPING = 'grasping'
    PLACING = 'placing'
    RETURNING = 'returning'


class StateMachine(Node):
    """任务状态机节点"""
    
    PATROL_POINTS = [
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 1.57),
        (0.0, 1.0, 3.14),
        (0.0, 0.0, 0.0),
    ]
    
    PLACE_POSITIONS = {
        0: (0.5, -0.5, 0.0),
        1: (-0.5, -0.5, 0.0),
    }
    
    def __init__(self):
        super().__init__('task_manager')
        
        self.current_state = TaskState.IDLE
        self.target_class = 0
        self.target_pose = None
        self.current_patrol_index = 0
        self.task_active = False
        self.goal_handle = None
        
        # 时间戳用于非阻塞等待
        self._state_start_time = None
        
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.detection_sub = self.create_subscription(
            PoseStamped, '/target_confirmed', self.detection_callback, 10
        )
        self.class_sub = self.create_subscription(
            Int32, '/detection/class', self.class_callback, 10
        )
        self.tracking_sub = self.create_subscription(
            Bool, '/tracking_status', self.tracking_callback, 10
        )
        self.task_cmd_sub = self.create_subscription(
            String, '/task/command', self.task_command_callback, 10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_close_pub = self.create_publisher(Bool, '/gripper/close', 10)
        self.gripper_open_pub = self.create_publisher(Bool, '/gripper/open', 10)
        self.state_pub = self.create_publisher(String, '/task/state', 10)
        
        self.timer = self.create_timer(0.1, self.state_machine_tick)
        
        self.get_logger().info('Task State Machine started')
    
    def task_command_callback(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == 'START':
            self.start_task()
        elif cmd == 'STOP':
            self.stop_task()
        elif cmd == 'PATROL':
            self.start_patrol()
        elif cmd == 'HOME':
            self.go_home()
    
    def start_task(self):
        self.task_active = True
        self.current_state = TaskState.PATROL
        self.current_patrol_index = 0
        self.navigate_to_patrol_point()
        self.get_logger().info('Task started')
    
    def stop_task(self):
        self.task_active = False
        self.current_state = TaskState.IDLE
        self.stop_navigation()
        self.get_logger().info('Task stopped')
    
    def start_patrol(self):
        self.current_state = TaskState.PATROL
        self.navigate_to_patrol_point()
    
    def go_home(self):
        self.navigate_to(0.0, 0.0, 0.0)
    
    def state_machine_tick(self):
        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)
        
        if not self.task_active:
            return
        
        if self.current_state == TaskState.PATROL:
            self.handle_patrol()
        elif self.current_state == TaskState.DETECTED:
            self.handle_detected()
        elif self.current_state == TaskState.TRACKING:
            self.handle_tracking()
        elif self.current_state == TaskState.APPROACHING:
            self.handle_approaching()
        elif self.current_state == TaskState.GRASPING:
            self.handle_grasping()
        elif self.current_state == TaskState.PLACING:
            self.handle_placing()
        elif self.current_state == TaskState.RETURNING:
            self.handle_returning()
    
    def handle_patrol(self):
        pass
    
    def handle_detected(self):
        self.stop_navigation()
        self.current_state = TaskState.TRACKING
        self.get_logger().info('Switching to TRACKING')
    
    def handle_tracking(self):
        if self.target_pose is None:
            return
        
        dx = self.target_pose.pose.position.x
        dy = self.target_pose.pose.position.y
        dz = self.target_pose.pose.position.z
        
        cmd = Twist()
        cmd.linear.x = min(0.3, dz * 0.5)
        cmd.linear.y = dx * 0.5
        cmd.angular.z = -dx * 0.5
        self.cmd_vel_pub.publish(cmd)
        
        distance = (dx**2 + dy**2 + dz**2) ** 0.5
        if distance < 0.3:
            self.current_state = TaskState.APPROACHING
            self.get_logger().info('Switching to APPROACHING')
    
    def handle_approaching(self):
        self.stop_navigation()
        self.current_state = TaskState.GRASPING
        self._state_start_time = self.get_clock().now()
        self.get_logger().info('Switching to GRASPING')
    
    def handle_grasping(self):
        if self._state_start_time is None:
            self._state_start_time = self.get_clock().now()
            grip_msg = Bool()
            grip_msg.data = True
            self.gripper_close_pub.publish(grip_msg)
            self.get_logger().info('Closing gripper')
            return
        
        elapsed = (self.get_clock().now() - self._state_start_time).nanoseconds / 1e9
        if elapsed >= 1.0:
            self._state_start_time = None
            place_pos = self.PLACE_POSITIONS.get(self.target_class, (0.5, -0.5, 0.0))
            self.navigate_to(*place_pos)
            self.current_state = TaskState.PLACING
            self.get_logger().info(f'Switching to PLACING (class={self.target_class})')
    
    def handle_placing(self):
        if self._state_start_time is None:
            self._state_start_time = self.get_clock().now()
            return
        
        elapsed = (self.get_clock().now() - self._state_start_time).nanoseconds / 1e9
        if elapsed >= 2.0:
            grip_msg = Bool()
            grip_msg.data = True
            self.gripper_open_pub.publish(grip_msg)
            self.get_logger().info('Opening gripper')
            
            self._state_start_time = None
            self.target_confirmed = False
            self.current_state = TaskState.PATROL
            self.navigate_to_patrol_point()
            self.get_logger().info('Switching to PATROL')
    
    def handle_returning(self):
        self.current_state = TaskState.PATROL
    
    def detection_callback(self, msg):
        if self.current_state == TaskState.PATROL:
            self.target_pose = msg
            self.current_state = TaskState.DETECTED
            self.stop_navigation()
            self.get_logger().info('Target detected!')
    
    def class_callback(self, msg):
        self.target_class = msg.data
    
    def tracking_callback(self, msg):
        pass
    
    def navigate_to_patrol_point(self):
        if self.current_patrol_index >= len(self.PATROL_POINTS):
            self.current_patrol_index = 0
        x, y, yaw = self.PATROL_POINTS[self.current_patrol_index]
        self.navigate_to(x, y, yaw)
        self.current_patrol_index += 1
    
    def navigate_to(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self._nav_client.wait_for_server(timeout_sec=5.0)
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)
        
        self.get_logger().info(f'Navigating to ({x:.1f}, {y:.1f}, {yaw:.1f})')
    
    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)
    
    def _on_nav_result(self, future):
        result = future.result()
        self.get_logger().info(f'Navigation result: {result.status}')
    
    def stop_navigation(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_task()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
