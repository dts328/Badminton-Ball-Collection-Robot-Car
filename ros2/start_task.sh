#!/bin/bash
# ============================================
# 一键任务脚本
# 启动完整机器人系统执行抓取任务
# ============================================

echo "=========================================="
echo "  Smart Robot - Task Execution"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Sourcing ROS2 Humble...${NC}"
    source /opt/ros/humble/setup.bash
fi

# 检查工作空间
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# 设置串口权限
echo -e "${GREEN}Setting serial port permissions...${NC}"
sudo chmod 666 /dev/ttyACM0 2>/dev/null
sudo chmod 666 /dev/ttyACM1 2>/dev/null
sudo chmod 666 /dev/ttyACM2 2>/dev/null

# 检查地图文件
MAP_FILE="src/robot_bringup/maps/warehouse.yaml"
if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}Error: Map file not found: $MAP_FILE${NC}"
    exit 1
fi

# 启动所有节点
echo -e "${GREEN}Starting all robot nodes...${NC}"
echo ""

echo -e "${YELLOW}[1/9] Starting Robot State Publisher...${NC}"
ros2 launch robot_description display.launch.py &
sleep 2

echo -e "${YELLOW}[2/9] Starting LD06 Lidar...${NC}"
ros2 run ldlidar_driver ldlidar_node --ros-args \
    -p port_name:=/dev/ttyACM2 \
    -p port_baudrate:=230400 &
sleep 2

echo -e "${YELLOW}[3/9] Starting Chassis Driver...${NC}"
ros2 run chassis_driver serial_bridge --ros-args \
    -p port_name:=/dev/ttyACM0 \
    -p baudrate:=115200 &
sleep 1

echo -e "${YELLOW}[4/9] Starting Odometry Publisher...${NC}"
ros2 run chassis_driver odom_publisher &
sleep 1

echo -e "${YELLOW}[5/9] Starting Orbbec Camera...${NC}"
ros2 run orbbec_camera orbbec_node &
sleep 2

echo -e "${YELLOW}[6/9] Starting RDK X5 Inference...${NC}"
ros2 run rdkx5_inference bpu_inference &
sleep 2

echo -e "${YELLOW}[7/9] Starting Vision Bridge...${NC}"
ros2 run vision_bridge rdkx5_receiver &
ros2 run vision_bridge depth_converter &
ros2 run vision_bridge target_tracker &
sleep 2

echo -e "${YELLOW}[8/9] Starting Arm Controller...${NC}"
ros2 run arm_controller moveit_client --ros-args \
    -p port_name:=/dev/ttyACM1 &
sleep 1

echo -e "${YELLOW}[9/9] Starting Navigation + Task Manager...${NC}"
ros2 launch robot_bringup nav2.launch.py map:=$MAP_FILE &
sleep 3

ros2 run task_manager state_machine &
sleep 2

echo ""
echo -e "${GREEN}=========================================="
echo "  All Nodes Started!"
echo "=========================================="
echo ""
echo "  Task Commands:"
echo "    - Start task:  ros2 topic pub /task/command std_msgs/msg/String \"data: START\""
echo "    - Stop task:   ros2 topic pub /task/command std_msgs/msg/String \"data: STOP\""
echo "    - Go home:     ros2 topic pub /task/command std_msgs/msg/String \"data: HOME\""
echo ""
echo "  System Status:"
echo "    - ros2 topic echo /task/state"
echo "    - ros2 topic echo /detection/class"
echo "    - ros2 topic echo /battery_voltage"
echo ""
echo "  To stop all nodes:"
echo "    ./stop_all.sh"
echo -e "${NC}"

# 保持运行
wait
