# 羽毛球收集机器人小车

**[English Documentation / 英文文档](README.md)**

一套集视觉识别、自主导航与精准抓取于一体的智能羽毛球收集机器人系统。本项目实现了从感知到执行的完整三层架构。

---

## 概述

本项目是一个用于室内羽毛球场馆的智能羽毛球收集机器人，集成了：

- **视觉识别**：基于地平线RDK X5的YOLOv8-Pose模型，BPU推理
- **自主导航**：SLAM建图、AMCL定位、Nav2路径规划
- **精准控制**：麦克纳姆轮底盘 + 5+1自由度机械臂

### 核心特性

- 二分类检测：完好球（class 0）/ 破损球（class 1）
- mAP50: 0.995，15帧滑窗时序投票
- 全向移动麦克纳姆轮底盘
- 10ms控制周期，PID + 电压前馈补偿
- 实时BPU推理，约30 FPS

---

## 系统架构

系统采用三层架构：**边缘视觉 - ROS2决策 - 嵌入式控制**

### 第一层：边缘视觉（RDK X5）
- 在BPU上运行YOLOv8-Pose模型（10 TOPS算力）
- 处理640x640分辨率的摄像头输入
- 输出检测结果：边界框、关键点、类别ID
- 实现15帧滑窗投票机制，确保判断稳定

### 第二层：ROS2决策（PC/边缘计算机）
- 处理SLAM建图与定位
- 使用Nav2栈进行导航规划
- 通过MoveIt2控制机械臂
- 使用状态机协调任务执行

### 第三层：嵌入式控制（STM32F407）
- PID控制4个麦克纳姆轮
- 读取编码器和IMU数据
- 监测电池电压
- 通过二进制协议通信

### 数据流

```
摄像头 -> RDK X5 (BPU) -> 检测结果 -> ROS2
                                           |
激光雷达 -> ROS2 (SLAM/Nav2) -> 速度指令 -> STM32
                                           |
                                      编码器反馈
```

---

## 硬件配置

### 主要组件

| 组件 | 型号 | 规格 |
|------|------|------|
| 主控 | STM32F407VET6 | 168MHz, ARM Cortex-M4 |
| 视觉计算 | 地平线RDK X5 | 10 TOPS BPU |
| 激光雷达 | 乐动LD06 | 2D, 12m量程 |
| 深度相机 | 奥比中光 | RGB-D |
| 底盘 | 麦克纳姆轮×4 | 直径97mm |
| 机械臂 | 5+1自由度 | 舵机驱动 |

### 串口配置

| 设备 | 端口 | 波特率 | 协议 |
|------|------|--------|------|
| STM32底盘 | /dev/ttyACM0 | 115200 | 二进制 |
| STM32机械臂 | /dev/ttyACM1 | 115200 | 文本 |
| LD06激光雷达 | /dev/ttyACM2 | 230400 | 二进制 |

### 机械参数

| 参数 | 数值 | 说明 |
|------|------|------|
| 轮子直径 | 97mm | 麦克纳姆轮 |
| 轮子厚度 | 44mm | 麦克纳姆轮 |
| 轮距（前后） | 120mm | 前后轮距 |
| 轮距（左右） | 236mm | 左右轮距 |
| 雷达高度 | 200mm | 离地高度 |
| IMU位置 | 中心后50mm | 陀螺仪位置 |

---

## 项目结构

```
Badminton-Ball-Collection-Robot-Car/
|
|-- stm32/                          STM32嵌入式控制
|   |-- Core/
|   |   |-- Inc/                    头文件
|   |   |   |-- main.h              主头文件
|   |   |   |-- pid.h               PID控制器
|   |   |   |-- mecanum.h           麦克纳姆轮运动学
|   |   |   |-- motor.h             电机驱动
|   |   |   |-- encoder.h           编码器接口
|   |   |   |-- protocol.h          文本协议
|   |   |   |-- ros_protocol.h      二进制协议
|   |   |   |-- battery.h           电池监测
|   |   |   |-- kinematics.h        机械臂运动学
|   |   |   |-- servo.h             舵机控制
|   |   |   |-- imu.h               IMU接口
|   |   |   `-- ...                 其他头文件
|   |   |
|   |   `-- Src/                    源文件
|   |       |-- main.c              主循环 (10ms)
|   |       |-- pid.c               增量式PID
|   |       |-- mecanum.c           运动学解算
|   |       |-- motor.c             PWM输出
|   |       |-- encoder.c           编码器读取
|   |       |-- protocol.c          文本协议
|   |       |-- ros_protocol.c      二进制协议
|   |       `-- ...                 其他源文件
|   |
|   |-- Drivers/                    HAL库
|   `-- MDK-ARM/                    Keil工程
|
|-- ros2/                           ROS2工作空间
|   |-- src/
|   |   |-- robot_bringup/          启动文件和配置
|   |   |-- robot_description/      URDF模型
|   |   |-- chassis_driver/         底盘串口通信
|   |   |-- ldlidar_driver/         LD06激光雷达驱动
|   |   |-- orbbec_camera/          奥比中光深度相机
|   |   |-- vision_bridge/          视觉结果处理
|   |   |-- arm_controller/         机械臂控制
|   |   |-- task_manager/           任务状态机
|   |   |-- rdkx5_inference/        BPU推理
|   |   |-- robot_interfaces/       自定义消息/服务
|   |   `-- teleop/                 手动控制
|   |
|   |-- build.sh                    构建脚本
|   |-- start_slam.sh               SLAM建图
|   |-- start_nav.sh                导航
|   |-- start_task.sh               完整任务
|   `-- stop_all.sh                 停止所有
|
|-- README.md                       英文文档
`-- README_cn.md                    中文文档
```

---

## ROS2包说明

### robot_bringup
系统启动文件和配置文件，包含Nav2、AMCL、SLAM的参数配置。

### robot_description
机器人URDF模型，描述底盘、轮子、激光雷达、摄像头和机械臂的物理结构。

### chassis_driver
处理与STM32的串口通信，使用二进制协议实现高效数据传输。

### ldlidar_driver
LD06 2D激光雷达驱动，以10Hz频率发布LaserScan消息。

### orbbec_camera
奥比中光深度相机驱动，提供RGB图像、深度图和点云数据。

### vision_bridge
处理RDK X5的视觉检测结果，实现15帧滑窗投票机制，确保目标确认稳定可靠。

### arm_controller
通过串口控制5+1自由度机械臂，支持预设姿态（归零、准备、抓取、放置）。

### task_manager
实现任务状态机，状态流转：IDLE -> PATROL -> DETECTED -> TRACKING -> APPROACHING -> GRASPING -> PLACING。

### rdkx5_inference
在RDK X5 BPU上运行YOLOv8-Pose推理，输出检测结果（类别ID、置信度）。

### robot_interfaces
自定义消息、服务和动作定义，用于节点间通信。

### teleop
键盘和手柄控制，用于调试和建图时的手动操作。

---

## 快速开始

### 环境要求

1. 安装ROS2 Humble
```bash
sudo apt install ros-humble-desktop
```

2. 安装依赖包
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-moveit
sudo apt install ros-humble-orbbec-camera
```

3. 安装Python包
```bash
pip install pyserial numpy opencv-python
```

### 编译

```bash
cd ros2
./build.sh
source install/setup.bash
```

### 运行

#### SLAM建图
```bash
# 终端1：启动SLAM
./start_slam.sh

# 终端2：手动控制机器人
ros2 run teleop keyboard_teleop
```

#### 自主导航
```bash
./start_nav.sh
# 在RViz2中设置初始位姿和导航目标
```

#### 完整任务执行
```bash
./start_task.sh
# 机器人将巡逻、检测并收集羽毛球
```

#### 停止所有
```bash
./stop_all.sh
```

---

## 通信协议

### 二进制协议（ROS2 <-> STM32）

底盘使用二进制协议实现高效通信。

#### 接收帧（ROS2 -> STM32，11字节）

```
[0x7B][Mode][Res][X_H][X_L][Y_H][Y_L][Z_H][Z_L][校验][0x7D]
```

| 字节 | 内容 | 说明 |
|------|------|------|
| 0 | 0x7B | 帧头 |
| 1 | Mode | 0x00=底盘, 0x01=机械臂 |
| 2 | 保留 | 0x00 |
| 3-4 | X速度 | 大端序int16, mm/s |
| 5-6 | Y速度 | 大端序int16, mm/s |
| 7-8 | Z速度 | 大端序int16, mm/s |
| 9 | 校验 | XOR校验 |
| 10 | 0x7D | 帧尾 |

#### 发送帧（STM32 -> ROS2，26字节）

```
[0x7B][flags][encA][encB][encC][encD][accX][accY][accZ]
[gyroX][gyroY][gyroZ][bat_H][bat_L][校验][0x7D]
```

| 偏移 | 内容 | 单位 |
|------|------|------|
| 1 | 标志位 | bit0: 停止标志 |
| 2-9 | 编码器A-D | mm/s, 大端序int16 |
| 10-15 | 加速度X/Y/Z | 原始值 |
| 16-21 | 陀螺仪X/Y/Z | 原始值 |
| 22-23 | 电池电压 | mV, 大端序uint16 |

### 文本协议（调试用）

系统也支持文本命令用于调试：

| 命令 | 格式 | 说明 |
|------|------|------|
| 速度 | $SPD:vx,vy,w! | m/s |
| 机械臂 | $ARM:j1,j2,j3,j4,j5,grip! | 角度 |
| PID | $MPID:kp,ki,kd! | 设置PID参数 |
| 停止 | $MSTOP! | 紧急停止 |
| 状态 | $MSTATUS! | 查询状态 |

---

## 性能指标

| 指标 | 数值 | 说明 |
|------|------|------|
| 控制周期 | 10ms | 100Hz PID循环 |
| 最大速度 | 1.0 m/s | 线速度 |
| 最大角速度 | 2.0 rad/s | 旋转速度 |
| 速度精度 | ±0.01 m/s | 稳态误差 |
| 检测mAP50 | 0.995 | 球分类精度 |
| 推理帧率 | ~30 FPS | BPU推理速度 |
| 投票窗口 | 15帧 | 滑窗大小 |
| 投票阈值 | 8帧 | 确认所需最小帧数 |

---

## 开发指南

### 添加新功能

1. 创建新的ROS2包：
```bash
cd ros2/src
ros2 pkg create --build-type ament_python my_package
```

2. 在package.xml中添加依赖

3. 实现你的Python节点

4. 更新setup.py

5. 编译测试：
```bash
cd ros2
colcon build --packages-select my_package
source install/setup.bash
ros2 run my_package my_node
```

### 调试方法

#### 查看话题
```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /detection/class
```

#### 查看TF树
```bash
ros2 run tf2_tools view_frames
```

#### 监控系统
```bash
ros2 node list
ros2 run rqt_graph rqt_graph
```

---

## 常见问题

### 串口权限不足
```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
sudo chmod 666 /dev/ttyACM2
```

### 摄像头未找到
```bash
# 检查USB连接
lsusb
# 检查视频设备
ls /dev/video*
```

### 激光雷达无数据
```bash
# 检查串口连接
screen /dev/ttyACM2 230400
# 应该能看到二进制数据流
```

---

## 许可证

本项目采用MIT许可证。

---

## 致谢

- ROS2社区提供的优秀机器人框架
- 地平线机器人提供的RDK X5平台
- Ultralytics提供的YOLOv8模型
