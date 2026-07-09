#!/usr/bin/env python3
"""
Nav2 Navigation Launch File
启动Nav2导航�?+ AMCL定位 + 全部驱动节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路�?    bringup_dir = get_package_share_directory('robot_bringup')
    description_dir = get_package_share_directory('robot_description')
    
    # 配置文件
    nav2_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    amcl_params = os.path.join(bringup_dir, 'config', 'amcl_params.yaml')
    controller_params = os.path.join(bringup_dir, 'config', 'controller.yaml')
    urdf_file = os.path.join(description_dir, 'urdf', 'robot.urdf')
    
    # 地图文件
    map_file = os.path.join(bringup_dir, 'maps', 'my_map.yaml')
    
    # 读取URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map', default=map_file)
    autostart = LaunchConfiguration('autostart', default='true')
    
    # ==================== 驱动节点 ====================
    
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
    
    # 底盘驱动
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
    
    # 里程�?    odom_publisher = Node(
        package='chassis_driver',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 激光雷�?    ldlidar_node = Node(
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
    
    # ==================== 定位节点 ====================
    
    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # AMCL定位
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # 定位生命周期管理
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl']}
        ]
    )
    
    # ==================== 导航节点 ====================
    
    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            controller_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[
            nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # 导航生命周期管理
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]}
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
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('autostart', default_value='true'),
        
        # 驱动节点
        robot_state_publisher,
        joint_state_publisher,
        chassis_driver,
        odom_publisher,
        ldlidar_node,
        
        # 定位节点
        map_server,
        amcl_node,
        localization_lifecycle,
        
        # 导航节点
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        navigation_lifecycle,
        
        # 可视�?        rviz2,
    ])
