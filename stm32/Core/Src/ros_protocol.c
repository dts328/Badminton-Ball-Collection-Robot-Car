/**
  ******************************************************************************
  * @file    ros_protocol.c
  * @brief   ROS binary protocol implementation
  ******************************************************************************
  */

#include "ros_protocol.h"
#include "mecanum.h"
#include "imu.h"
#include "battery.h"
#include "motor.h"
#include "pid.h"
#include "usart.h"
#include "kinematics.h"
#include "servo.h"
#include <string.h>
#include <stdio.h>

/* Exported variables --------------------------------------------------------*/
volatile uint8_t ros_rx_flag = 0;
ROS_RxBuffer_t ros_rx_buf;
float ROS_Move_X = 0, ROS_Move_Y = 0, ROS_Move_Z = 0;
uint8_t ROS_Control_Mode = ROS_MODE_MECANUM;

/* Private variables ---------------------------------------------------------*/
static uint8_t ros_rx_count = 0;
static uint8_t ros_rx_temp[ROS_RX_FRAME_LEN];

/* External UART handle */
extern UART_HandleTypeDef huart3;

/**
  * @brief  Initialize ROS protocol
  */
void ROS_Init(void)
{
    ros_rx_flag = 0;
    ros_rx_count = 0;
    ROS_Move_X = 0;
    ROS_Move_Y = 0;
    ROS_Move_Z = 0;
    ROS_Control_Mode = ROS_MODE_MECANUM;
    memset(ros_rx_temp, 0, sizeof(ros_rx_temp));
    memset(ros_rx_buf.buffer, 0, sizeof(ros_rx_buf.buffer));
    
    /* Enable USART3 RXNE interrupt */
    USART3->CR1 |= USART_CR1_RXNEIE;
}

/**
  * @brief  Calculate XOR checksum
  */
uint8_t ROS_CalcChecksum(uint8_t *buf, uint8_t len)
{
    uint8_t i, checksum = 0;
    for(i = 0; i < len; i++)
    {
        checksum ^= buf[i];
    }
    return checksum;
}

/**
  * @brief  Process received byte from USART3 (state machine)
  * @param  data: Received byte
  * @note   Call this function for each byte received from USART3
  *         States: WAIT_HEADER -> READ_PAYLOAD -> VERIFY
  */
void ROS_ProcessReceive(uint8_t data)
{
    static uint8_t ros_state = 0;  /* 0=WAIT_HEADER, 1=READ_PAYLOAD */
    
    switch(ros_state)
    {
        case 0:  /* WAIT_HEADER */
            if(data == ROS_FRAME_HEADER)
            {
                ros_rx_temp[0] = data;
                ros_rx_count = 1;
                ros_state = 1;
            }
            break;
            
        case 1:  /* READ_PAYLOAD */
            ros_rx_temp[ros_rx_count] = data;
            ros_rx_count++;
            
            if(ros_rx_count >= ROS_RX_FRAME_LEN)
            {
                /* Frame complete, verify */
                ros_rx_count = 0;
                ros_state = 0;
                
                /* Verify frame tail */
                if(ros_rx_temp[ROS_RX_FRAME_LEN - 1] == ROS_FRAME_TAIL)
                {
                    /* Verify checksum */
                    if(ros_rx_temp[ROS_RX_FRAME_LEN - 2] == ROS_CalcChecksum(ros_rx_temp, ROS_RX_FRAME_LEN - 2))
                    {
                        /* Copy to receive buffer */
                        memcpy(ros_rx_buf.buffer, ros_rx_temp, ROS_RX_FRAME_LEN);
                        
                        /* Set receive flag */
                        ros_rx_flag = 1;
                    }
                }
            }
            break;
            
        default:
            ros_state = 0;
            ros_rx_count = 0;
            break;
    }
}

/**
  * @brief  Clamp float value to range
  */
static float clampf(float value, float min, float max)
{
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

/**
  * @brief  Read big-endian 16-bit signed integer from buffer
  */
static int16_t ROS_ReadBE16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

/**
  * @brief  Write big-endian 16-bit unsigned value to buffer
  */
static void ROS_WriteBE16U(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFF);
}

/**
  * @brief  Write big-endian 16-bit signed value to buffer
  */
static void ROS_WriteBE16(uint8_t *buf, int16_t val)
{
    buf[0] = (uint8_t)((uint16_t)val >> 8);
    buf[1] = (uint8_t)((uint16_t)val & 0xFF);
}

/**
  * @brief  Handle mecanum control mode
  * @note   Manual buffer parsing (no struct mapping) with big-endian
  */
static void ROS_HandleMecanum(void)
{
    /* Parse big-endian 16-bit values from buffer */
    /* Frame: [0x7B][Mode][Res][X_H][X_L][Y_H][Y_L][Z_H][Z_L][Chk][0x7D] */
    int16_t x = ROS_ReadBE16(ros_rx_buf.buffer[3], ros_rx_buf.buffer[4]);
    int16_t y = ROS_ReadBE16(ros_rx_buf.buffer[5], ros_rx_buf.buffer[6]);
    int16_t z = ROS_ReadBE16(ros_rx_buf.buffer[7], ros_rx_buf.buffer[8]);
    
    /* Convert mm/s to m/s */
    ROS_Move_X = (float)x / 1000.0f;
    ROS_Move_Y = (float)y / 1000.0f;
    ROS_Move_Z = (float)z / 1000.0f;
    
    /* Velocity clamping */
    ROS_Move_X = clampf(ROS_Move_X, -1.0f, 1.0f);
    ROS_Move_Y = clampf(ROS_Move_Y, -1.0f, 1.0f);
    ROS_Move_Z = clampf(ROS_Move_Z, -2.0f, 2.0f);
}

/**
  * @brief  Handle robotic arm control mode
  * @note   Manual buffer parsing (no struct mapping)
  *         Frame: [0x7B][0x01][Res][J1_H][J1_L][J2_H][J2_L][J3_H][J3_L][Chk][0x7D]
  *         Unit: 0.01 degree (int16)
  */
static void ROS_HandleArm(void)
{
    /* Parse big-endian 16-bit values from buffer */
    int16_t j1_raw = ROS_ReadBE16(ros_rx_buf.buffer[3], ros_rx_buf.buffer[4]);
    int16_t j2_raw = ROS_ReadBE16(ros_rx_buf.buffer[5], ros_rx_buf.buffer[6]);
    int16_t j3_raw = ROS_ReadBE16(ros_rx_buf.buffer[7], ros_rx_buf.buffer[8]);
    
    /* Convert to angles (degrees) */
    float j1 = (float)j1_raw / 100.0f;  /* 0.01 degree precision */
    float j2 = (float)j2_raw / 100.0f;
    float j3 = (float)j3_raw / 100.0f;
    
    /* Clamp angles to safe range */
    j1 = clampf(j1, -180.0f, 180.0f);
    j2 = clampf(j2, -180.0f, 180.0f);
    j3 = clampf(j3, -180.0f, 180.0f);
    
    /* Execute inverse kinematics */
    extern kinematics_t kinematics;
    extern int kinematics_move(float x, float y, float z, kinematics_t *kin);
    extern void servo_move_arm(int16_t *pwm, uint16_t time_ms);
    
    /* Convert angles to approximate XYZ coordinates */
    /* Simple mapping: J1->X, J2->Y, J3->Z */
    float x = j1 * 2.0f;  /* Scale factor */
    float y = j2 * 2.0f;
    float z = j3 * 2.0f + 150.0f;  /* Offset for height */
    
    /* Execute IK and move arm */
    int result = kinematics_move(x, y, z, &kinematics);
    if(result)
    {
        servo_move_arm(kinematics.servo_pwm, 500);
    }
}

/**
  * @brief  Process ROS receive data
  * @note   Call this in main loop after ros_rx_flag is set
  */
void ROS_ProcessData(void)
{
    if(ros_rx_flag == 0) return;
    
    ros_rx_flag = 0;
    
    /* Check control mode - use buffer index instead of struct */
    ROS_Control_Mode = ros_rx_buf.buffer[1];
    
    switch(ROS_Control_Mode)
    {
        case ROS_MODE_MECANUM:
            ROS_HandleMecanum();
            break;
            
        case ROS_MODE_ARM:
            ROS_HandleArm();
            break;
            
        case ROS_MODE_AUTO_CHARGE:
            /* Auto charging mode - not implemented */
            break;
            
        default:
            break;
    }
}

/**
  * @brief  Send status frame to ROS (26 bytes, big-endian)
  * @note   Call this at 20Hz (50ms interval)
  *         Frame format:
  *         [0x7B][flags][A_H][A_L][B_H][B_L][C_H][C_L][D_H][D_L]
  *         [accX_H][accX_L][accY_H][accY_L][accZ_H][accZ_L]
  *         [gyroX_H][gyroX_L][gyroY_H][gyroY_L][gyroZ_H][gyroZ_L]
  *         [bat_H][bat_L][checksum][0x7D]
  */
void ROS_SendStatus(void)
{
    uint8_t buf[26];
    uint8_t i;
    uint8_t checksum = 0;
    
    /* Frame header */
    buf[0] = ROS_FRAME_HEADER;
    
    /* Flags (bit0: stop flag) */
    buf[1] = (Motor_IsEnabled()) ? 0x00 : 0x01;
    
    /* Four wheel speeds (mm/s, big-endian) */
    ROS_WriteBE16(&buf[2], (int16_t)(Encoder_A * 1000));
    ROS_WriteBE16(&buf[4], (int16_t)(Encoder_B * 1000));
    ROS_WriteBE16(&buf[6], (int16_t)(Encoder_C * 1000));
    ROS_WriteBE16(&buf[8], (int16_t)(Encoder_D * 1000));
    
    /* IMU accelerometer data (raw) */
    ROS_WriteBE16(&buf[10], imu.accel.x);
    ROS_WriteBE16(&buf[12], imu.accel.y);
    ROS_WriteBE16(&buf[14], imu.accel.z);
    
    /* IMU gyroscope data (raw) */
    ROS_WriteBE16(&buf[16], imu.gyro.x);
    ROS_WriteBE16(&buf[18], imu.gyro.y);
    ROS_WriteBE16(&buf[20], imu.gyro.z);
    
    /* Battery voltage (*1000, big-endian, unsigned) */
    ROS_WriteBE16U(&buf[22], (uint16_t)(Voltage * 1000));
    
    /* Calculate checksum (XOR of bytes 0-23) */
    for(i = 0; i < 24; i++)
    {
        checksum ^= buf[i];
    }
    buf[24] = checksum;
    
    /* Frame tail */
    buf[25] = ROS_FRAME_TAIL;
    
    /* Send frame */
    for(i = 0; i < 26; i++)
    {
        /* Wait for USART3 TX ready */
        while(!(USART3->SR & USART_SR_TXE));
        USART3->DR = buf[i];
    }
}

/**
  * @brief  USART3 send single byte
  */
void ROS_SendByte(uint8_t data)
{
    USART3->DR = data;
    while(!(USART3->SR & USART_SR_TXE));
}
