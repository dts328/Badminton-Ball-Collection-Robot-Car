# STM32 二进制通信协议说明

## 概述

STM32与ROS2之间使用二进制协议通信，相比文本协议具有以下优势：
- 更高的传输效率
- 更小的数据包
- 更好的错误检测

## 协议格式

### 帧结构

| 字节 | 内容 | 说明 |
|------|------|------|
| 0 | 帧头 | 固定 0x7B |
| 1 | 模式 | 0x00=底盘, 0x01=机械臂 |
| 2 | 保留 | 0x00 |
| 3-4 | X速度/J1 | 大端序int16 |
| 5-6 | Y速度/J2 | 大端序int16 |
| 7-8 | Z速度/J3 | 大端序int16 |
| 9 | 校验 | XOR校验 |
| 10 | 帧尾 | 固定 0x7D |

### 接收帧 (ROS2 -> STM32)

#### 底盘控制帧 (11字节)
```
[0x7B][0x00][0x00][X_H][X_L][Y_H][Y_L][Z_H][Z_L][CHK][0x7D]
```

- X/Y/Z: 速度值，单位 mm/s，大端序int16
- 限幅: -32768 ~ 32767

**示例**: 前进 0.5 m/s
```
X = 0.5 * 1000 = 500 = 0x01F4
帧: 7B 00 00 01 F4 00 00 00 00 XX 7D
```

### 发送帧 (STM32 -> ROS2)

#### 状态帧 (26字节)
```
[0x7B][flags][A_H][A_L][B_H][B_L][C_H][C_L][D_H][D_L]
[accX_H][accX_L][accY_H][accY_L][accZ_H][accZ_L]
[gyroX_H][gyroX_L][gyroY_H][gyroY_L][gyroZ_H][gyroZ_L]
[bat_H][bat_L][checksum][0x7D]
```

| 偏移 | 内容 | 单位 | 说明 |
|------|------|------|------|
| 1 | flags | - | bit0: 停止标志 |
| 2-9 | 编码器A-D | mm/s | 大端序int16 |
| 10-15 | 加速度X/Y/Z | 原始值 | 大端序int16 |
| 16-21 | 陀螺仪X/Y/Z | 原始值 | 大端序int16 |
| 22-23 | 电池电压 | mV | 大端序uint16 |

## 速度转换

### ROS2 -> STM32
```python
# m/s -> mm/s
x_mm = int(vx_ms * 1000)

# 打包为大端序int16
buf[3:5] = struct.pack('>h', x_mm)
```

### STM32 -> ROS2
```python
# mm/s -> m/s
vx_ms = struct.unpack('>h', buf[2:4])[0] / 1000.0

# mV -> V
voltage = struct.unpack('>H', buf[22:24])[0] / 1000.0
```

## 校验计算

```python
def calc_checksum(buf):
    """计算XOR校验 (不包括校验位和帧尾)"""
    checksum = 0
    for b in buf[:-2]:
        checksum ^= b
    return checksum
```

## 麦克纳姆轮运动学

### 正运动学 (编码器 -> 机体速度)
```python
vx = (enc_a + enc_b + enc_c + enc_d) / 4
vy = (-enc_a + enc_b + enc_c - enc_d) / 4
vw = (-enc_a + enc_b - enc_c + enc_d) / (4 * (Lx + Ly))
```

### 逆运动学 (机体速度 -> 轮速)
```python
omega_a = (vx - vy - (Lx + Ly) * vw) / R
omega_b = (vx + vy + (Lx + Ly) * vw) / R
omega_c = (vx + vy - (Lx + Ly) * vw) / R
omega_d = (vx - vy + (Lx + Ly) * vw) / R
```

## ROS2代码示例

### 发送底盘控制
```python
from chassis_driver.serial_bridge import build_mecanum_frame

# 构建帧
frame = build_mecanum_frame(vx=0.5, vy=0.0, vz=0.0)

# 发送
serial_port.write(frame)
```

### 解析状态帧
```python
from chassis_driver.odom_publisher import parse_status_frame

# 解析
status = parse_status_frame(frame_data)

if status:
    encoders = status['encoders']  # [A, B, C, D] m/s
    battery = status['battery']    # V
```

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| /cmd_vel | Twist | 速度指令 (m/s) |
| /chassis_status | UInt8MultiArray | 二进制状态帧 |
| /battery_voltage | Float32 | 电池电压 (V) |
| /odom | Odometry | 里程计 |

## 故障排查

### 1. 无法通信
- 检查串口是否正确: `/dev/ttyACM0`
- 检查波特率: 115200
- 检查串口权限: `sudo chmod 666 /dev/ttyACM0`

### 2. 数据解析错误
- 检查帧头帧尾: 0x7B, 0x7D
- 检查校验和是否正确
- 检查字节序: 大端序

### 3. 速度不正确
- 检查单位转换: m/s -> mm/s
- 检查限幅: -32768 ~ 32767
- 检查运动学参数是否正确
