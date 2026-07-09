#!/bin/bash
# ============================================
# 一键构建脚本
# 编译ROS2工作空间
# ============================================

echo "=========================================="
echo "  Building ROS2 Workspace"
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

# 清理旧的构建
echo -e "${YELLOW}Cleaning old build...${NC}"
rm -rf build/ install/ log/

# 安装依赖
echo -e "${YELLOW}Installing dependencies...${NC}"
rosdep install --from-paths src --ignore-src -r -y

# 编译
echo -e "${GREEN}Building workspace...${NC}"
colcon build --symlink-install

# 检查编译结果
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}=========================================="
    echo "  Build Successful!"
    echo "=========================================="
    echo ""
    echo "  To use the workspace:"
    echo "    source install/setup.bash"
    echo ""
    echo "  Quick start commands:"
    echo "    ./start_slam.sh   - Start SLAM mapping"
    echo "    ./start_nav.sh    - Start navigation"
    echo "    ./start_task.sh   - Start full task"
    echo "    ./stop_all.sh     - Stop all nodes"
    echo -e "${NC}"
else
    echo ""
    echo -e "${RED}=========================================="
    echo "  Build Failed!"
    echo "=========================================="
    echo "  Please check the error messages above."
    echo -e "${NC}"
    exit 1
fi
