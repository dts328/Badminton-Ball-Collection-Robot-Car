# Badminton Ball Collection Robot Car

**[Chinese Documentation / 中文文档](README_cn.md)**

An intelligent robot system for badminton ball collection, featuring visual recognition, autonomous navigation, and precise grasping. This project implements a complete three-layer architecture from perception to execution.

---

## Overview

This project is a smart robot designed for collecting badminton balls in indoor courts. It combines:

- **Visual Recognition**: YOLOv8-Pose model running on Horizon RDK X5 with BPU inference
- **Autonomous Navigation**: SLAM mapping, AMCL localization, and Nav2 path planning
- **Precision Control**: Mecanum wheel chassis with 5+1 DOF robotic arm

### Key Features

- Two-class detection: Perfect ball (class 0) / Damaged ball (class 1)
- mAP50: 0.995 with 15-frame sliding window voting
- Full omnidirectional movement with mecanum wheels
- 10ms control cycle with PID and voltage feedforward
- Real-time BPU inference at ~30 FPS

---

## System Architecture

The system adopts a three-layer architecture: **Edge Vision - ROS2 Decision - Embedded Control**

### Layer 1: Edge Vision (RDK X5)
- Runs YOLOv8-Pose model on BPU (10 TOPS)
- Processes camera input at 640x640 resolution
- Outputs detection results: bounding box, keypoints, class ID
- Implements 15-frame sliding window voting for robust judgment

### Layer 2: ROS2 Decision (PC/Edge Computer)
- Handles SLAM mapping and localization
- Manages navigation with Nav2 stack
- Controls robotic arm via MoveIt2
- Coordinates task execution with state machine

### Layer 3: Embedded Control (STM32F407)
- Controls 4 mecanum wheels with PID
- Reads encoders and IMU data
- Manages battery voltage monitoring
- Communicates via binary protocol

### Data Flow

```
Camera -> RDK X5 (BPU) -> Detection Result -> ROS2
                                                    |
Lidar -> ROS2 (SLAM/Nav2) -> Velocity Command -> STM32
                                                    |
                                            Encoder Feedback
```

---

## Hardware Configuration

### Main Components

| Component | Model | Specification |
|-----------|-------|---------------|
| Main Controller | STM32F407VET6 | 168MHz, ARM Cortex-M4 |
| Vision Processor | Horizon RDK X5 | 10 TOPS BPU |
| Lidar | LD06 | 2D, 12m range |
| Depth Camera | Orbbec | RGB-D |
| Chassis | Mecanum Wheel x4 | 97mm diameter |
| Robot Arm | 5+1 DOF | Servo driven |

### Serial Port Configuration

| Device | Port | Baudrate | Protocol |
|--------|------|----------|----------|
| STM32 Chassis | /dev/ttyACM0 | 115200 | Binary |
| STM32 Arm | /dev/ttyACM1 | 115200 | Text |
| LD06 Lidar | /dev/ttyACM2 | 230400 | Binary |

### Mechanical Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Wheel Diameter | 97mm | Mecanum wheel |
| Wheel Width | 44mm | Mecanum wheel |
| Wheelbase | 120mm | Front-rear distance |
| Wheel Track | 236mm | Left-right distance |
| Lidar Height | 200mm | From ground |
| IMU Offset | -50mm | Behind center |

---

## Project Structure

```
Badminton-Ball-Collection-Robot-Car/
|
|-- stm32/                          STM32 embedded control
|   |-- Core/
|   |   |-- Inc/                    Header files
|   |   |   |-- main.h              Main header
|   |   |   |-- pid.h               PID controller
|   |   |   |-- mecanum.h           Mecanum kinematics
|   |   |   |-- motor.h             Motor driver
|   |   |   |-- encoder.h           Encoder interface
|   |   |   |-- protocol.h          Text protocol
|   |   |   |-- ros_protocol.h      Binary protocol
|   |   |   |-- battery.h           Battery monitor
|   |   |   |-- kinematics.h        Arm kinematics
|   |   |   |-- servo.h             Servo control
|   |   |   |-- imu.h               IMU interface
|   |   |   `-- ...                 Other headers
|   |   |
|   |   `-- Src/                    Source files
|   |       |-- main.c              Main loop (10ms)
|   |       |-- pid.c               Incremental PID
|   |       |-- mecanum.c           Kinematics
|   |       |-- motor.c             PWM output
|   |       |-- encoder.c           Encoder reading
|   |       |-- protocol.c          Text protocol
|   |       |-- ros_protocol.c      Binary protocol
|   |       `-- ...                 Other sources
|   |
|   |-- Drivers/                    HAL library
|   `-- MDK-ARM/                    Keil project
|
|-- ros2/                           ROS2 workspace
|   |-- src/
|   |   |-- robot_bringup/          Launch files and configs
|   |   |-- robot_description/      URDF model
|   |   |-- chassis_driver/         Chassis serial communication
|   |   |-- ldlidar_driver/         LD06 lidar driver
|   |   |-- orbbec_camera/          Orbbec depth camera
|   |   |-- vision_bridge/          Vision result processing
|   |   |-- arm_controller/         Arm control
|   |   |-- task_manager/           Task state machine
|   |   |-- rdkx5_inference/        BPU inference
|   |   |-- robot_interfaces/       Custom messages/services
|   |   `-- teleop/                 Manual control
|   |
|   |-- build.sh                    Build script
|   |-- start_slam.sh               SLAM mapping
|   |-- start_nav.sh                Navigation
|   |-- start_task.sh               Full task
|   `-- stop_all.sh                 Stop all
|
|-- README.md                       This file
`-- README_cn.md                    Chinese documentation
```

---

## ROS2 Packages

### robot_bringup
Launch files and configuration files for the complete system.

### robot_description
URDF model describing the robot's physical structure, including chassis, wheels, lidar, camera, and robotic arm.

### chassis_driver
Handles serial communication with STM32 for chassis control. Uses binary protocol for high-efficiency data transfer.

### ldlidar_driver
Driver for LD06 2D lidar, publishing LaserScan messages at 10Hz.

### orbbec_camera
Driver for Orbbec depth camera, providing RGB images, depth maps, and point clouds.

### vision_bridge
Processes vision detection results from RDK X5, implements 15-frame sliding window voting for robust target confirmation.

### arm_controller
Controls the 5+1 DOF robotic arm via serial communication, supports preset poses (home, ready, grasp, place).

### task_manager
Implements task state machine with states: IDLE -> PATROL -> DETECTED -> TRACKING -> APPROACHING -> GRASPING -> PLACING.

### rdkx5_inference
Runs YOLOv8-Pose inference on RDK X5 BPU, outputs detection results with class ID and confidence.

### robot_interfaces
Custom message, service, and action definitions for inter-node communication.

### teleop
Keyboard and joystick control for manual operation during debugging and mapping.

---

## Quick Start

### Prerequisites

1. Install ROS2 Humble
```bash
sudo apt install ros-humble-desktop
```

2. Install dependencies
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-moveit
sudo apt install ros-humble-orbbec-camera
```

3. Install Python packages
```bash
pip install pyserial numpy opencv-python
```

### Build

```bash
cd ros2
./build.sh
source install/setup.bash
```

### Run

#### SLAM Mapping
```bash
# Terminal 1: Start SLAM
./start_slam.sh

# Terminal 2: Control robot manually
ros2 run teleop keyboard_teleop
```

#### Autonomous Navigation
```bash
./start_nav.sh
# Use RViz2 to set initial pose and navigation goals
```

#### Full Task Execution
```bash
./start_task.sh
# Robot will patrol, detect balls, and collect them
```

#### Stop All
```bash
./stop_all.sh
```

---

## Communication Protocol

### Binary Protocol (ROS2 <-> STM32)

The chassis uses a binary protocol for efficient communication.

#### Receive Frame (ROS2 -> STM32, 11 bytes)

```
[0x7B][Mode][Res][X_H][X_L][Y_H][Y_L][Z_H][Z_L][Checksum][0x7D]
```

| Byte | Content | Description |
|------|---------|-------------|
| 0 | 0x7B | Frame header |
| 1 | Mode | 0x00=chassis, 0x01=arm |
| 2 | Reserved | 0x00 |
| 3-4 | X speed | Big-endian int16, mm/s |
| 5-6 | Y speed | Big-endian int16, mm/s |
| 7-8 | Z speed | Big-endian int16, mm/s |
| 9 | Checksum | XOR of bytes 0-8 |
| 10 | 0x7D | Frame tail |

#### Send Frame (STM32 -> ROS2, 26 bytes)

```
[0x7B][flags][encA][encB][encC][encD][accX][accY][accZ]
[gyroX][gyroY][gyroZ][bat_H][bat_L][Checksum][0x7D]
```

| Offset | Content | Unit |
|--------|---------|------|
| 1 | Flags | bit0: stop flag |
| 2-9 | Encoder A-D | mm/s, big-endian int16 |
| 10-15 | Accelerometer X/Y/Z | Raw value |
| 16-21 | Gyroscope X/Y/Z | Raw value |
| 22-23 | Battery voltage | mV, big-endian uint16 |

### Text Protocol (Debug)

For debugging, the system also supports text commands:

| Command | Format | Description |
|---------|--------|-------------|
| Velocity | $SPD:vx,vy,w! | m/s |
| Arm | $ARM:j1,j2,j3,j4,j5,grip! | degrees |
| PID | $MPID:kp,ki,kd! | Set PID params |
| Stop | $MSTOP! | Emergency stop |
| Status | $MSTATUS! | Query status |

---

## Performance Metrics

| Metric | Value | Description |
|--------|-------|-------------|
| Control Cycle | 10ms | 100Hz PID loop |
| Max Speed | 1.0 m/s | Linear velocity |
| Max Angular | 2.0 rad/s | Rotational velocity |
| Speed Accuracy | ±0.01 m/s | Steady-state error |
| Detection mAP50 | 0.995 | Ball classification |
| Inference FPS | ~30 | BPU inference speed |
| Voting Window | 15 frames | Sliding window size |
| Vote Threshold | 8 frames | Minimum votes for confirmation |

---

## Development

### Adding New Features

1. Create a new ROS2 package:
```bash
cd ros2/src
ros2 pkg create --build-type ament_python my_package
```

2. Add dependencies to package.xml

3. Implement your node in Python

4. Update CMakeLists.txt or setup.py

5. Build and test:
```bash
cd ros2
colcon build --packages-select my_package
source install/setup.bash
ros2 run my_package my_node
```

### Debugging

#### View topics
```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /detection/class
```

#### View TF tree
```bash
ros2 run tf2_tools view_frames
```

#### Monitor system
```bash
ros2 node list
ros2 run rqt_graph rqt_graph
```

---

## Troubleshooting

### Serial port permission denied
```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
sudo chmod 666 /dev/ttyACM2
```

### Camera not found
```bash
# Check USB connection
lsusb
# Check video devices
ls /dev/video*
```

### Lidar no data
```bash
# Check serial connection
screen /dev/ttyACM2 230400
# Should see binary data streaming
```

---

## License

This project is licensed under the MIT License.

---

## Acknowledgments

- ROS2 community for the excellent robotics framework
- Horizon Robotics for RDK X5 platform
- Ultralytics for YOLOv8 model
