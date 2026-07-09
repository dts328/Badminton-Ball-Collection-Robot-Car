# Robot State Message
# 机器人整体状态消息

# 消息头
std_msgs/Header header

# 任务状态
string task_state          # idle/patrol/detected/tracking/approaching/grasping/placing

# 底盘状态
float32 battery_voltage    # 电池电压 (V)
float32 vx                 # X轴速度 (m/s)
float32 vy                 # Y轴速度 (m/s)
float32 vz                 # Z轴角速度 (rad/s)
float32[4] wheel_speeds    # 四轮转速 (rad/s)

# 机械臂状态
float32[6] joint_angles    # 关节角度 (rad)
bool gripper_state         # 夹爪状态 true=打开 false=关闭

# 传感器状态
bool lidar_ok              # 激光雷达状态
bool camera_ok             # 相机状态
bool imu_ok                # IMU状态

# 导航状态
float32[2] current_pose    # 当前位姿 (x, y)
float32[2] target_pose     # 目标位姿 (x, y)
bool is_navigating         # 是否正在导航
