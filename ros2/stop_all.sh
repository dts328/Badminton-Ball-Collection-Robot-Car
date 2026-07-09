#!/bin/bash
# ============================================
# 停止所有节点脚本
# ============================================

echo "=========================================="
echo "  Stopping all robot nodes..."
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 停止ROS2节点
echo -e "${YELLOW}Killing ROS2 nodes...${NC}"

# 停止所有相关进程
pkill -f "ros2 run"
pkill -f "ros2 launch"
pkill -f "slam_toolbox"
pkill -f "nav2"
pkill -f "amcl"
pkill -f "rviz2"
pkill -f "chassis_driver"
pkill -f "ldlidar"
pkill -f "orbbec"
pkill -f "vision_bridge"
pkill -f "arm_controller"
pkill -f "task_manager"
pkill -f "rdkx5_inference"

# 等待进程退出
sleep 2

# 检查是否还有残留进程
if pgrep -f "ros2" > /dev/null; then
    echo -e "${RED}Some ROS2 processes still running, forcing kill...${NC}"
    pkill -9 -f "ros2"
fi

echo -e "${GREEN}=========================================="
echo "  All nodes stopped!"
echo "==========================================${NC}"
