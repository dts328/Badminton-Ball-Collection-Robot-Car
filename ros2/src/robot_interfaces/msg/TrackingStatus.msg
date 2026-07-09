# Tracking Status Message
# 目标追踪状态消息

# 消息头
std_msgs/Header header

# 追踪状态
bool is_tracking             # 是否正在追踪
bool is_confirmed            # 目标是否已确认
int32 track_id               # 追踪ID

# 投票统计
int32 window_size            # 滑窗大小
int32 vote_count             # 投票数
float32 vote_ratio           # 投票比例

# 目标信息
int32 target_class           # 目标类别
float32 target_confidence    # 目标置信度

# 3D位姿
geometry_msgs/PoseStamped target_pose

# 追踪质量
float32 tracking_quality     # 追踪质量 (0-1)
int32 lost_frames            # 丢失帧数
