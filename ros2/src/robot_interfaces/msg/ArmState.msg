# Arm State Message
# 机械臂状态消息

# 消息头
std_msgs/Header header

# 关节状态
float32[6] joint_positions     # 关节位置 (rad)
float32[6] joint_velocities    # 关节速度 (rad/s)
float32[6] joint_torques       # 关节力矩 (N·m)

# 末端执行器状态
geometry_msgs/Pose end_effector_pose  # 末端位姿
bool gripper_open                      # 夹爪状态
float32 gripper_force                  # 夹爪力 (N)

# 运动状态
bool is_moving                 # 是否正在运动
bool has_error                 # 是否有错误
string error_message           # 错误信息

# 预定义姿态
string current_preset          # 当前预姿态: home/ready/grasp/place
