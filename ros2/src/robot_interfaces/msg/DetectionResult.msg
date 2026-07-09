# Detection Result Message
# 视觉检测结果消息

# 消息头
std_msgs/Header header

# 目标信息
int32 class_id             # 类别ID: 0=完好球 1=破损球
string class_name          # 类别名称: PERFECT/DAMAGED
float32 confidence         # 置信度 (0-1)

# 2D边界框
int32 bbox_x               # 边界框左上角x
int32 bbox_y               # 边界框左上角y
int32 bbox_width           # 边界框宽度
int32 bbox_height          # 边界框高度

# 3D位姿 (相机坐标系)
geometry_msgs/Pose pose_3d

# 关键点 (YOLOv8-Pose)
geometry_msgs/Point[5] keypoints

# 时间戳
int64 timestamp_ns         # 纳秒时间戳
