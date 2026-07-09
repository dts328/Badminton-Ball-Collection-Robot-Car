#!/bin/bash
# ============================================
# 一键导航脚本
# 启动Nav2导航栈 + AMCL定位
# ============================================

echo "=========================================="
echo "  Smart Robot - Navigation"
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
    echo -e "${YELLOW}Please run start_slam.sh first to create a map.${NC}"
    exit 1
fi

# 启动节点
echo -e "${GREEN}Starting Navigation nodes...${NC}"
echo ""

echo -e "${YELLOW}[1/6] Starting Robot State Publisher...${NC}"
ros2 launch robot_description display.launch.py &
sleep 2

echo -e "${YELLOW}[2/6] Starting LD06 Lidar...${NC}"
ros2 run ldlidar_driver ldlidar_node --ros-args \
    -p port_name:=/dev/ttyACM2 \
    -p port_baudrate:=230400 &
sleep 2

echo -e "${YELLOW}[3/6] Starting Chassis Driver...${NC}"
ros2 run chassis_driver serial_bridge --ros-args \
    -p port_name:=/dev/ttyACM0 \
    -p baudrate:=115200 &
sleep 1

echo -e "${YELLOW}[4/6] Starting Odometry Publisher...${NC}"
ros2 run chassis_driver odom_publisher &
sleep 1

echo -e "${YELLOW}[5/6] Starting Map Server + AMCL...${NC}"
ros2 launch robot_bringup nav2.launch.py map:=$MAP_FILE &
sleep 3

echo -e "${YELLOW}[6/6] Starting RViz2...${NC}"
ros2 run rviz2 rviz2 -d src/robot_bringup/rviz/robot.rviz &
sleep 2

echo ""
echo -e "${GREEN}=========================================="
echo "  Navigation Started!"
echo "=========================================="
echo ""
echo "  Controls:"
echo "    - Use RViz2 to set initial pose"
echo "    - Use RViz2 'Nav2 Goal' to set navigation target"
echo "    - Robot will navigate autonomously"
echo ""
echo "  To stop all nodes:"
echo "    ./stop_all.sh"
echo -e "${NC}"

# 保持运行
wait
