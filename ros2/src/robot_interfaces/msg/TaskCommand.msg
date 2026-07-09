# Task Command Message
# 任务指令消息

# 指令类型
string command              # START/STOP/PATROL/HOME/GRASP/PLACE/RESET

# 目标参数 (可选)
int32 target_class          # 目标类别: 0=完好球 1=破损球
geometry_msgs/PoseStamped target_pose  # 目标位姿

# 优先级
int8 priority               # 优先级: 0=低 1=中 2=高
