#!/usr/bin/env python3
"""
SLAM Launch File
启动SLAM建图所需的全部节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('robot_bringup')
    description_dir = get_package_share_directory('robot_description')
    
    # 配置文件
    slam_params = os.path.join(bringup_dir, 'config', 'slam_params.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'robot.urdf')
    
    # 读取URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
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
    
    # SLAM Toolbox Node
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # RViz2
    rviz_config = os.path.join(bringup_dir, 'rviz', 'robot.rviz')
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
        
        # 启动所有节点
        robot_state_publisher,
        joint_state_publisher,
        chassis_driver,
        odom_publisher,
        ldlidar_node,
        slam_toolbox,
        rviz2,
    ])
