# Smart Robot ROS2 Workspace

智能视觉定位与抓取机器人系统ROS2工作空间

## 系统概述

本系统是一个集视觉识别、自主导航与精准抓取于一体的智能机器人平台，采用"边缘视觉-ROS2决策-嵌入式控制"三层架构。

## 硬件配置

| 组件 | 型号 | 串口 |
|------|------|------|
| 主控 | STM32F407VET6 | - |
| 底盘通信 | UART | /dev/ttyACM0 |
| 机械臂通信 | UART | /dev/ttyACM1 |
| 激光雷达 | 乐动LD06 | /dev/ttyACM2 |
| 深度相机 | 奥比中光 | USB |
| 视觉计算 | 地平线RDK X5 | - |

## 工作空间结构

```
robot_ws/
├── src/
│   ├── robot_bringup/          # 启动文件和配置
│   ├── robot_description/      # 机器人URDF描述
│   ├── chassis_driver/         # 底盘驱动
│   ├── ldlidar_driver/         # LD06激光雷达驱动
│   ├── orbbec_camera/          # 奥比中光深度相机
│   ├── vision_bridge/          # 视觉桥接
│   ├── arm_controller/         # 机械臂控制
│   ├── task_manager/           # 任务调度
│   └── rdkx5_inference/        # RDK X5推理
```

## 快速开始

### 1. 构建工作空间

```bash
cd robot_ws
./build.sh
source install/setup.bash
```

### 2. 一键建图

```bash
./start_slam.sh
```

使用键盘控制机器人移动：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3. 一键导航

```bash
./start_nav.sh
```

在RViz2中使用Nav2 Goal设置导航目标。

### 4. 一键执行任务

```bash
./start_task.sh
```

发送任务指令：
```bash
# 开始任务
ros2 topic pub /task/command std_msgs/msg/String "data: START"

# 停止任务
ros2 topic pub /task/command std_msgs/msg/String "data: STOP"
```

### 5. 停止所有节点

```bash
./stop_all.sh
```

## 节点说明

### chassis_driver
- `serial_bridge`: 串口通信桥接，与STM32通信
- `odom_publisher`: 里程计发布，麦克纳姆轮正运动学

### ldlidar_driver
- `ldlidar_node`: LD06激光雷达驱动

### orbbec_camera
- `orbbec_node`: 奥比中光深度相机驱动
- `pointcloud`: 点云处理

### vision_bridge
- `rdkx5_receiver`: RDK X5识别结果接收
- `depth_converter`: 深度图转3D坐标
- `target_tracker`: 15帧滑窗投票追踪

### arm_controller
- `moveit_client`: MoveIt2接口
- `gripper_control`: 夹爪控制

### task_manager
- `state_machine`: 任务状态机
- `behavior_tree`: 行为树

### rdkx5_inference
- `bpu_inference`: BPU推理节点

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| /scan | LaserScan | 激光扫描数据 |
| /odom | Odometry | 里程计 |
| /cmd_vel | Twist | 速度指令 |
| /camera/color/image_raw | Image | 彩色图像 |
| /camera/depth/image_raw | Image | 深度图像 |
| /camera/depth/points | PointCloud2 | 点云 |
| /detection/result | PoseStamped | 检测结果 |
| /target_confirmed | PoseStamped | 确认目标 |
| /task/state | String | 任务状态 |
| /battery_voltage | Float32 | 电池电压 |

## 配置文件

| 文件 | 说明 |
|------|------|
| config/nav2_params.yaml | Nav2导航参数 |
| config/amcl_params.yaml | AMCL定位参数 |
| config/slam_params.yaml | SLAM建图参数 |
| config/controller.yaml | 控制器参数 |
| config/moveit_params.yaml | MoveIt2参数 |

## 常见问题

### 串口权限
```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
sudo chmod 666 /dev/ttyACM2
```

### 检查节点状态
```bash
ros2 node list
ros2 topic list
ros2 service list
```

### 查看话题数据
```bash
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /task/state
```

## 开发团队

- Robot Development Team

## 许可证

Apache-2.0
