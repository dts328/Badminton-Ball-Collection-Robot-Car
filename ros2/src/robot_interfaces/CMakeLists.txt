cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  # 自定义消息
  "msg/RobotState.msg"
  "msg/DetectionResult.msg"
  "msg/ArmState.msg"
  "msg/TaskCommand.msg"
  "msg/TrackingStatus.msg"
  
  # 自定义服务
  "srv/TriggerDetection.srv"
  "srv/SetArmPose.srv"
  "srv/ControlGripper.srv"
  "srv/NavigateTo.srv"
  
  # 自定义动作
  "action/Navigate.action"
  "action/Grasp.action"
  "action/Patrol.action"
  
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()
