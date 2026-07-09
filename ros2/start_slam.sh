#!/bin/bash
# ============================================
# 一键建图脚本
# 使用SLAM Toolbox建立环境地图
# ============================================

echo "=========================================="
echo "  Smart Robot - SLAM Mapping"
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

# 启动节点
echo -e "${GREEN}Starting SLAM nodes...${NC}"
echo ""

echo -e "${YELLOW}[1/4] Starting Robot State Publisher...${NC}"
ros2 launch robot_description display.launch.py &
sleep 2

echo -e "${YELLOW}[2/4] Starting LD06 Lidar...${NC}"
ros2 run ldlidar_driver ldlidar_node --ros-args \
    -p port_name:=/dev/ttyACM2 \
    -p port_baudrate:=230400 &
sleep 2

echo -e "${YELLOW}[3/4] Starting Chassis Driver...${NC}"
ros2 run chassis_driver serial_bridge --ros-args \
    -p port_name:=/dev/ttyACM0 \
    -p baudrate:=115200 &
sleep 1

echo -e "${YELLOW}[4/4] Starting SLAM Toolbox...${NC}"
ros2 launch slam_toolbox online_async_launch.py \
    params_file:=src/robot_bringup/config/slam_params.yaml &
sleep 2

echo ""
echo -e "${GREEN}=========================================="
echo "  SLAM Mapping Started!"
echo "=========================================="
echo ""
echo "  Controls:"
echo "    - Use RViz2 to set initial pose"
echo "    - Use 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'"
echo "      to control the robot"
echo "    - Map will be saved automatically"
echo ""
echo "  To save map manually:"
echo "    ros2 run nav2_map_server map_saver_cli -f maps/my_map"
echo ""
echo "  To stop all nodes:"
echo "    ./stop_all.sh"
echo -e "${NC}"

# 等待用户输入
read -p "Press Enter to save map and exit..."

# 保存地图
echo -e "${GREEN}Saving map...${NC}"
ros2 run nav2_map_server map_saver_cli -f maps/warehouse

echo -e "${GREEN}Map saved to maps/warehouse.yaml${NC}"
echo -e "${GREEN}Done!${NC}"
