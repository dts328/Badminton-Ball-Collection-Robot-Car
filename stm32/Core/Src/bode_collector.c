/**
  ******************************************************************************
  * @file    bode_collector.c
  * @brief   波特图数据采集模块 - 离线记录模式
  * @note    先存入RAM，采集完再一次性发送，消除串口阻塞
  ******************************************************************************
  */

#include "bode_collector.h"
#include "motor.h"
#include "encoder.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 波特图配置 */
BodeConfig_t bode_config;

/* 离线记录缓冲区 (36KB) */
BodeData_t bode_buffer[BODE_MAX_SAMPLES];
uint16_t bode_buffer_idx = 0;

/* 外部变量 */
extern float Encoder_A, Encoder_B, Encoder_C, Encoder_D;
extern UART_HandleTypeDef huart1;

/**
  * @brief  初始化波特图采集模块
  */
void Bode_Init(void)
{
    memset(&bode_config, 0, sizeof(bode_config));
    bode_config.state = BODE_IDLE;
    bode_buffer_idx = 0;
}

/**
  * @brief  开始波特图采集（离线记录模式）
  */
void Bode_Start(float freq, float pwm_amp, float pwm_bias, uint16_t samples)
{
    bode_config.freq = freq;
    bode_config.pwm_amp = pwm_amp;
    bode_config.pwm_bias = pwm_bias;
    bode_config.samples = samples;
    bode_config.sample_idx = 0;
    bode_config.state = BODE_RUNNING;
    bode_buffer_idx = 0;
    
    /* 清空缓冲区 */
    memset(bode_buffer, 0, sizeof(bode_buffer));
}

/**
  * @brief  停止波特图采集并发送数据
  */
void Bode_Stop(void)
{
    bode_config.state = BODE_DONE;
    
    /* 停止电机 */
    Set_Pwm(0, 0, 0, 0);
    
    /* 一次性发送所有数据 */
    Bode_SendAllData();
}

/**
  * @brief  获取当前PWM值（开环正弦波）
  */
float Bode_GetPWM(void)
{
    if (bode_config.state != BODE_RUNNING) {
        return 0.0f;
    }
    
    float t = (float)bode_config.sample_idx / (float)BODE_SAMPLE_RATE;
    float pwm = bode_config.pwm_bias + bode_config.pwm_amp * sinf(2.0f * M_PI * bode_config.freq * t);
    
    return pwm;
}

/**
  * @brief  波特图处理函数（在5ms定时任务中调用）
  * @note   离线记录模式，开环控制+死区补偿
  */
void Bode_Process(void)
{
    if (bode_config.state != BODE_RUNNING) {
        return;
    }
    
    /* 检查是否完成 */
    if (bode_buffer_idx >= bode_config.samples) {
        Bode_Stop();
        return;
    }
    
    /* 1. 获取当前PWM值 */
    float pwm = Bode_GetPWM();
    
    /* 2. 死区补偿 */
    int pwm_out = (int)pwm;
    if(pwm_out > 0) pwm_out = pwm_out + DEAD_ZONE;
    if(pwm_out < 0) pwm_out = pwm_out - DEAD_ZONE;
    
    /* 3. 直接设置PWM（开环控制+死区补偿） */
    Set_Pwm(pwm_out, pwm_out, pwm_out, pwm_out);
    
    /* 4. 读取编码器速度 */
    extern void Get_Velocity_Form_Encoder(void);
    Get_Velocity_Form_Encoder();
    
    /* 5. 存入缓冲区（不发送！） */
    bode_buffer[bode_buffer_idx].timestamp = HAL_GetTick();
    bode_buffer[bode_buffer_idx].pwm = pwm;
    bode_buffer[bode_buffer_idx].speed_a = Encoder_A;
    bode_buffer[bode_buffer_idx].speed_b = Encoder_B;
    bode_buffer[bode_buffer_idx].speed_c = Encoder_C;
    bode_buffer[bode_buffer_idx].speed_d = Encoder_D;
    bode_buffer_idx++;
    bode_config.sample_idx++;
}

/**
  * @brief  一次性发送所有采集数据
  * @note   采集完成后调用，单独输出四个轮子数据
  */
void Bode_SendAllData(void)
{
    char msg[128];
    
    /* 发送头部 */
    snprintf(msg, sizeof(msg), "BODE_START\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    /* 发送参数 */
    snprintf(msg, sizeof(msg), "mode,offline\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    snprintf(msg, sizeof(msg), "freq,%.2f\r\n", bode_config.freq);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    snprintf(msg, sizeof(msg), "pwm_amp,%.1f\r\n", bode_config.pwm_amp);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    snprintf(msg, sizeof(msg), "pwm_bias,%.1f\r\n", bode_config.pwm_bias);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    snprintf(msg, sizeof(msg), "samples,%d\r\n", bode_buffer_idx);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    snprintf(msg, sizeof(msg), "sample_rate,%d\r\n", BODE_SAMPLE_RATE);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    snprintf(msg, sizeof(msg), "dead_zone,%d\r\n", DEAD_ZONE);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    /* 发送数据开始标记 */
    snprintf(msg, sizeof(msg), "DATA_START\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    /* 发送所有数据 - 单独输出四个轮子 */
    for(uint16_t i = 0; i < bode_buffer_idx; i++) {
        /* 格式: 时间戳,PWM,速度A,速度B,速度C,速度D */
        snprintf(msg, sizeof(msg), "%lu,%.1f,%.4f,%.4f,%.4f,%.4f\r\n",
                 bode_buffer[i].timestamp,
                 bode_buffer[i].pwm,
                 bode_buffer[i].speed_a,
                 bode_buffer[i].speed_b,
                 bode_buffer[i].speed_c,
                 bode_buffer[i].speed_d);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 50);
        HAL_Delay(1);  /* 避免串口溢出 */
    }
    
    /* 发送数据结束标记 */
    snprintf(msg, sizeof(msg), "DATA_END\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    snprintf(msg, sizeof(msg), "BODE_END\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
