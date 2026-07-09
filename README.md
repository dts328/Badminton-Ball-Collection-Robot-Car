# Badminton Ball Collection Robot Car

An intelligent robot system for badminton ball collection, featuring visual recognition, autonomous navigation, and precise grasping.

## System Overview

The system adopts a three-layer architecture: **Edge Vision - ROS2 Decision - Embedded Control**, forming a complete closed-loop from perception to execution.

```
┌─────────────────────────────────────────────────────────────────┐
│                    System Architecture                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐     UART      ┌─────────────┐    ROS2 DDS    │
│  │   RDK X5    │ ────────────→ │    ROS2     │ ←───────────→   │
│  │   (Vision)  │  Detection    │  (Decision) │    Topics       │
│  └─────────────┘               └──────┬──────┘                 │
│                                       │ UART                   │
│                                       ▼                        │
│                              ┌─────────────────┐               │
│                              │  STM32F407      │               │
│                              │  (Control)      │               │
│                              └────────┬────────┘               │
│                                       │ PWM/Encoder            │
│                                       ▼                        │
│                              ┌─────────────────┐               │
│                              │  Mecanum Car    │               │
│                              │  + Robot Arm    │               │
│                              └─────────────────┘               │
└─────────────────────────────────────────────────────────────────┘
```

## Project Structure

```
Badminton-Ball-Collection-Robot-Car/
├── stm32/                      # STM32 Embedded Control
│   ├── Core/
│   │   ├── Inc/                # Header files
│   │   └── Src/                # Source files
│   ├── Drivers/                # HAL Library
│   └── MDK-ARM/                # Keil Project
│
├── ros2/                       # ROS2 Workspace
│   ├── src/
│   │   ├── robot_bringup/      # Launch files
│   │   ├── robot_description/  # URDF model
│   │   ├── chassis_driver/     # Chassis driver
│   │   ├── ldlidar_driver/     # LD06 lidar
│   │   ├── orbbec_camera/      # Depth camera
│   │   ├── vision_bridge/      # Vision bridge
│   │   ├── arm_controller/     # Arm controller
│   │   ├── task_manager/       # Task scheduler
│   │   ├── rdkx5_inference/    # BPU inference
│   │   ├── robot_interfaces/   # Custom messages
│   │   └── teleop/             # Manual control
│   └── *.sh                    # Launch scripts
│
└── README.md
```

## Hardware Configuration

| Component | Model | Interface |
|-----------|-------|-----------|
| Main Controller | STM32F407VET6 | - |
| Vision Processor | Horizon RDK X5 | - |
| Lidar | LD06 | /dev/ttyACM2 @ 230400 |
| Depth Camera | Orbbec | USB |
| Chassis | Mecanum Wheel ×4 | /dev/ttyACM0 @ 115200 |
| Robot Arm | 5+1 DOF | /dev/ttyACM1 @ 115200 |

## Features

### Visual Recognition
- YOLOv8-Pose model with BPU inference
- 15-frame sliding window voting mechanism
- Two-class detection: Perfect (0) / Damaged (1)
- mAP50: 0.995

### Navigation
- SLAM mapping with slam_toolbox
- AMCL localization
- Nav2 path planning with TEB local planner
- Full omnidirectional movement support

### Control
- 10ms control cycle (100Hz)
- Incremental PID with voltage feedforward
- Mecanum wheel kinematics
- 5+1 DOF robotic arm control

## Quick Start

### STM32
1. Open `stm32/MDK-ARM/project.uvprojx` with Keil MDK
2. Build and flash to STM32F407

### ROS2
```bash
cd ros2

# Build workspace
./build.sh

# Start SLAM mapping
./start_slam.sh

# Start navigation
./start_nav.sh

# Start full task
./start_task.sh

# Stop all
./stop_all.sh
```

## Communication Protocol

### Binary Protocol (ROS2 ↔ STM32)

**Receive Frame (11 bytes)**:
```
[0x7B][Mode][Res][X_H][X_L][Y_H][Y_L][Z_H][Z_L][Chk][0x7D]
```

**Send Frame (26 bytes)**:
```
[0x7B][flags][encA][encB][encC][encD][accel][gyro][bat][Chk][0x7D]
```

## Performance

| Metric | Value |
|--------|-------|
| Control Cycle | 10ms (100Hz) |
| Max Speed | 1.0 m/s |
| Speed Accuracy | ±0.01 m/s |
| Detection mAP50 | 0.995 |
| Inference FPS | ~30 |

## License

MIT License
