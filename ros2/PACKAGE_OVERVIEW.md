# ROS2 工作空间包总览 (完整版)

## 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                           ROS2 Humble 工作空间                                   │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                        核心功能包                                        │    │
│  │  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────┐ │    │
│  │  │robot_     │ │robot_     │ │chassis_   │ │ldlidar_   │ │orbbec_    │ │    │
│  │  │bringup    │ │description│ │driver     │ │driver     │ │camera     │ │    │
│  │  │启动配置   │ │机器人描述 │ │底盘驱动   │ │激光雷达   │ │深度相机   │ │    │
│  │  └───────────┘ └───────────┘ └───────────┘ └───────────┘ └───────────┘ │    │
│  │  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────┐               │    │
│  │  │vision_    │ │arm_       │ │task_      │ │rdkx5_     │               │    │
│  │  │bridge     │ │controller │ │manager    │ │inference  │               │    │
│  │  │视觉桥接   │ │机械臂控制 │ │任务调度   │ │RDK X5推理 │               │    │
│  │  └───────────┘ └───────────┘ └───────────┘ └───────────┘               │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐    │
│  │                        接口与控制包                                      │    │
│  │  ┌───────────────────┐ ┌───────────────────┐                           │    │
│  │  │robot_interfaces   │ │teleop             │                           │    │
│  │  │自定义消息/服务/动作│ │键盘/手柄遥控      │                           │    │
│  │  └───────────────────┘ └───────────────────┘                           │    │
│  └─────────────────────────────────────────────────────────────────────────┘    │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 包详细信息

### 1. robot_bringup (启动配置包)

| 属性 | 值 |
|------|-----|
| 类型 | ament_cmake |
| 功能 | 系统启动和配置管理 |

**Launch文件**:
- `bringup.launch.py` - 全系统启动
- `nav2.launch.py` - Nav2+AMCL导航
- `slam.launch.py` - SLAM建图
- `display.launch.py` - 机器人显示

**配置文件**:
- `nav2_params.yaml` - Nav2导航参数
- `amcl_params.yaml` - AMCL定位参数
- `slam_params.yaml` - SLAM建图参数
- `controller.yaml` - 控制器参数
- `moveit_params.yaml` - MoveIt2参数

---

### 2. robot_description (机器人描述包)

| 属性 | 值 |
|------|-----|
| 类型 | ament_cmake |
| 功能 | 机器人URDF/SRDF描述 |

**URDF参数** (实际测量):
| 参数 | 值 | 说明 |
|------|-----|------|
| wheelbase | 0.12m | 轮子前后距离12cm |
| wheeltrack | 0.236m | 轮子左右距离23.6cm |
| wheel_radius | 0.0485m | 轮子直径9.7cm |
| wheel_width | 0.044m | 轮子厚度4.4cm |
| lidar_height | 0.2m | 雷达高度20cm |
| imu_x_offset | -0.05m | 陀螺仪后5cm |

**TF树**:
```
map -> odom -> base_link -> imu_frame -> laser_frame
                         -> orbbec_frame
                         -> camera_frame
                         -> arm_base -> link1~link5
                         -> wheel_fl/fr/bl/br
```

---

### 3. chassis_driver (底盘驱动包)

| 属性 | 值 |
|------|-----|
| 类型 | ament_python |
| 功能 | 底盘串口通信和里程计 |
| 串口 | /dev/ttyACM0 @ 115200 |
| 协议 | **二进制协议** (11字节/26字节) |

**节点**:
| 节点 | 功能 | 发布 | 订阅 |
|------|------|------|------|
| serial_bridge | 串口通信 | /chassis_status, /battery_voltage | /cmd_vel |
| odom_publisher | 里程计 | /odom, TF | /chassis_status |

**二进制协议**:
- 帧头: 0x7B, 帧尾: 0x7D
- 接收帧: `[0x7B][Mode][Res][X_H][X_L][Y_H][Y_L][Z_H][Z_L][Chk][0x7D]`
- 发送帧: 26字节 (编码器+IMU+电池)
- 速度单位: mm/s (int16, 大端序)
- 详细说明见 PROTOCOL.md

---

### 4. ldlidar_driver (激光雷达包)

| 属性 | 值 |
|------|-----|
| 类型 | ament_python |
| 功能 | LD06激光雷达驱动 |
| 串口 | /dev/ttyACM2 @ 230400 |

**节点**:
| 节点 | 功能 | 发布 |
|------|------|------|
| ldlidar_node | 激光扫描 | /scan |

---

### 5. orbbec_camera (深度相机包)

| 属性 | 值 |
|------|-----|
| 类型 | ament_python |
| 功能 | 奥比中光深度相机驱动 |

**节点**:
| 节点 | 功能 | 发布 |
|------|------|------|
| orbbec_node | 相机驱动 | /camera/color/image_raw, /camera/depth/image_raw |
| pointcloud | 点云处理 | /camera/depth/points |

**注意**: TF由robot_state_publisher从URDF发布

---

### 6. vision_bridge (视觉桥接包)

| 属性 | 值 |
|------|-----|
| 类型 | ament_python |
| 功能 | RDK X5结果接收和坐标转换 |

**节点**:
| 节点 | 功能 | 发布 | 订阅 |
|------|------|------|------|
| rdkx5_receiver | 接收识别结果 | /target_3d_pose, /detection/class | 串口数据 |
| depth_converter | 深度转3D | /target_3d_pose | /camera/depth/image_raw |
| target_tracker | 15帧滑窗 | /target_confirmed, /tracking_status | /target_3d_pose |

**数据流**:
```
RDK X5 -> rdkx5_receiver -> /target_3d_pose -> target_tracker -> /target_confirmed
深度图 -> depth_converter -> /target_3d_pose ->/
```

---

### 7. arm_controller (机械臂控制包)

| 属性 | 值 |
|------|-----|
| 类型 | ament_python |
| 功能 | 5+1自由度机械臂控制 |
| 串口 | /dev/ttyACM1 @ 115200 |

**节点**:
| 节点 | 功能 | 订阅 |
|------|------|------|
| moveit_client | 关节控制 | /arm_controller/joint_trajectory |
| gripper_control | 夹爪控制 | /gripper/open, /gripper/close |

**协议**:
- 关节: `$ARM:j1,j2,j3,j4,j5,grip!`
- 夹爪: `$GRIP:OPEN!` / `$GRIP:CLOSE!`

---

### 8. task_manager (任务调度包)

| 属性 | 值 |
|------|-----|
| 类型 | ament_python |
| 功能 | 任务状态机和行为树 |

**节点**:
| 节点 | 功能 | 发布 | 订阅 |
|------|------|------|------|
| state_machine | 任务调度 | /cmd_vel, /gripper/*, /task/state | /target_confirmed |
| behavior_tree | 行为树 | /cmd_vel, /bt/state | /tracking_status |

**状态机流程**:
```
IDLE -> PATROL -> DETECTED -> TRACKING -> APPROACHING -> GRASPING -> PLACING -> PATROL
```

---

### 9. rdkx5_inference (RDK X5推理包)

| 属性 | 值 |
|------|-----|
| 类型 | ament_python |
| 功能 | BPU推理和目标检测 |
| 模型 | yolov8n_pose_rdkx5.bin |

**节点**:
| 节点 | 功能 | 发布 | 订阅 |
|------|------|------|------|
| bpu_inference | BPU推理 | /detection/raw, /detection/pixel, /detection/class | 摄像头 |

**核心算法**:
- BPU推理: 使用hobot_dnn的pyeasy_dnn模块
- 后处理: DFL解码 + NMS非极大值抑制
- 投票机制: 15帧滑窗，≥8帧破损触发判定

**类别定义**:
- class 0: PERFECT (完好球)
- class 1: DAMAGED (破损球)

**检测流程**:
```
摄像头 -> 预处理 -> BPU推理 -> 9路输出解码 -> NMS -> 投票判定 -> 发布结果
```

**ROS话题**:
- /detection/raw: 原始检测结果
- /detection/pixel: 像素坐标
- /detection/class: 类别ID

---

### 10. robot_interfaces (自定义接口包) ⭐新增

| 属性 | 值 |
|------|-----|
| 类型 | ament_cmake |
| 功能 | 自定义消息/服务/动作接口 |

**消息 (msg)**:
| 消息 | 说明 |
|------|------|
| RobotState.msg | 机器人整体状态 |
| DetectionResult.msg | 视觉检测结果 |
| ArmState.msg | 机械臂状态 |
| TaskCommand.msg | 任务指令 |
| TrackingStatus.msg | 追踪状态 |

**服务 (srv)**:
| 服务 | 说明 |
|------|------|
| TriggerDetection.srv | 触发视觉识别 |
| SetArmPose.srv | 设置机械臂姿态 |
| ControlGripper.srv | 夹爪控制 |
| NavigateTo.srv | 导航到指定位置 |

**动作 (action)**:
| 动作 | 说明 |
|------|------|
| Navigate.action | 导航动作（带进度） |
| Grasp.action | 抓取动作（带进度） |
| Patrol.action | 巡航动作（带进度） |

---

### 11. teleop (遥控包) ⭐新增

| 属性 | 值 |
|------|-----|
| 类型 | ament_python |
| 功能 | 键盘/手柄遥控 |

**节点**:
| 节点 | 功能 | 发布 |
|------|------|------|
| keyboard_teleop | 键盘控制 | /cmd_vel, /gripper/*, /task/command |
| joy_teleop | 手柄控制 | /cmd_vel, /gripper/*, /task/command |

**键盘控制**:
```
移动: w/x(前后) a/d(左右) q/e(旋转) s(停止)
夹爪: o(打开) c(关闭)
任务: 1(开始) 2(停止) 3(巡航) 4(回起点)
```

---

## 话题总览

| 话题 | 类型 | 发布者 | 订阅者 |
|------|------|--------|--------|
| /scan | LaserScan | ldlidar_node | Nav2, AMCL |
| /odom | Odometry | odom_publisher | Nav2 |
| /cmd_vel | Twist | task_manager, teleop | serial_bridge |
| /camera/color/image_raw | Image | orbbec_node | bpu_inference |
| /camera/depth/image_raw | Image | orbbec_node | depth_converter |
| /detection/class | Int32 | bpu_inference | target_tracker |
| /target_3d_pose | PoseStamped | rdkx5_receiver | target_tracker |
| /target_confirmed | PoseStamped | target_tracker | state_machine |
| /task/state | String | state_machine | - |
| /gripper/close | Bool | state_machine, teleop | gripper_control |
| /gripper/open | Bool | state_machine, teleop | gripper_control |

---

## 快速启动

```bash
# 构建
./build.sh

# 建图 (需要先启动teleop控制机器人移动)
./start_slam.sh
# 另一个终端:
ros2 run teleop keyboard_teleop

# 导航
./start_nav.sh

# 任务
./start_task.sh

# 停止
./stop_all.sh
```

---

## 包依赖关系

```
robot_bringup ─────────┬──────────────────────────────────────┐
                       │                                      │
robot_description ─────┤                                      │
                       │                                      │
chassis_driver ────────┼──── robot_interfaces ─────────────────┤
                       │                                      │
ldlidar_driver ────────┤                                      │
                       │                                      │
orbbec_camera ─────────┤                                      │
                       │                                      │
vision_bridge ─────────┤                                      │
                       │                                      │
arm_controller ────────┤                                      │
                       │                                      │
task_manager ──────────┤                                      │
                       │                                      │
rdkx5_inference ───────┘                                      │
                                                              │
teleop ───────────────────────────────────────────────────────┘
```
