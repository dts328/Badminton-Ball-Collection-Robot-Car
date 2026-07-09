#!/usr/bin/env python3
"""
Bringup Launch File
启动所有核心节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('robot_bringup')
    description_dir = get_package_share_directory('robot_description')
    
    # 配置文件路径
    rviz_config = os.path.join(bringup_dir, 'rviz', 'robot.rviz')
    urdf_file = os.path.join(description_dir, 'urdf', 'robot.urdf')
    
    # 读取URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('rviz', default='false')
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 底盘驱动节点
    chassis_driver = Node(
        package='chassis_driver',
        executable='serial_bridge',
        name='chassis_serial_bridge',
        output='screen',
        parameters=[
            {'port_name': '/dev/ttyACM0'},
            {'baudrate': 115200},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 里程计发布节点
    odom_publisher = Node(
        package='chassis_driver',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # LD06激光雷达节点
    ldlidar_node = Node(
        package='ldlidar_driver',
        executable='ldlidar_node',
        name='ldlidar_node',
        output='screen',
        parameters=[
            {'port_name': '/dev/ttyACM2'},
            {'port_baudrate': 230400},
            {'topic_name': '/scan'},
            {'frame_id': 'laser_frame'},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 奥比中光深度相机节点
    orbbec_node = Node(
        package='orbbec_camera',
        executable='orbbec_node',
        name='orbbec_camera',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RDK X5视觉桥接节点
    rdkx5_receiver = Node(
        package='vision_bridge',
        executable='rdkx5_receiver',
        name='rdkx5_receiver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 深度相机坐标转换节点
    depth_converter = Node(
        package='vision_bridge',
        executable='depth_converter',
        name='depth_converter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 目标追踪器节点(15帧滑窗投票)
    target_tracker = Node(
        package='vision_bridge',
        executable='target_tracker',
        name='target_tracker',
        output='screen',
        parameters=[
            {'window_size': 15},
            {'vote_threshold': 8},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 机械臂控制节点（合并关节和夹爪）
    arm_controller = Node(
        package='arm_controller',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[
            {'port_name': '/dev/ttyACM1'},
            {'baudrate': 115200},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 任务调度节点
    task_manager = Node(
        package='task_manager',
        executable='state_machine',
        name='task_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RDK X5 BPU推理节点
    rdkx5_inference = Node(
        package='rdkx5_inference',
        executable='bpu_inference',
        name='rdkx5_inference',
        output='screen',
        parameters=[
            {'model_path': 'models/yolov8n_pose_rdkx5.bin'},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz2可视化（可选）
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        
        # 基础驱动
        robot_state_publisher,
        joint_state_publisher,
        chassis_driver,
        odom_publisher,
        ldlidar_node,
        orbbec_node,
        
        # 视觉处理
        rdkx5_receiver,
        depth_converter,
        target_tracker,
        
        # 机械臂
        arm_controller,
        
        # 任务管理
        task_manager,
        
        # BPU推理
        rdkx5_inference,
        
        # 可视化
        rviz2,
    ])
