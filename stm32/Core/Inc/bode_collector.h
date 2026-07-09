/**
  ******************************************************************************
  * @file    bode_collector.h
  * @brief   波特图数据采集模块 - 离线记录模式
  ******************************************************************************
  */

#ifndef __BODE_COLLECTOR_H
#define __BODE_COLLECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 波特图采集参数 */
#define BODE_SAMPLE_RATE        200     /* 采样率 Hz */
#define BODE_MAX_SAMPLES        1500    /* 最大采样点数 (36KB RAM) */
#define DEAD_ZONE               250     /* PWM死区阈值 (实测: 200-300) */

/* 波特图采集状态 */
typedef enum {
    BODE_IDLE = 0,
    BODE_RUNNING,
    BODE_DONE
} BodeState_t;

/* 离线记录数据结构 */
typedef struct {
    uint32_t timestamp;     /* 时间戳 (ms) */
    float pwm;             /* PWM值 */
    float speed_a;         /* A轮速度 (m/s) */
    float speed_b;         /* B轮速度 (m/s) */
    float speed_c;         /* C轮速度 (m/s) */
    float speed_d;         /* D轮速度 (m/s) */
} BodeData_t;

/* 波特图采集参数结构 */
typedef struct {
    float freq;             /* 频率 Hz */
    float pwm_amp;          /* PWM振幅 */
    float pwm_bias;         /* PWM偏置 */
    uint16_t samples;       /* 采样点数 */
    uint16_t sample_idx;    /* 当前采样索引 */
    BodeState_t state;      /* 采集状态 */
} BodeConfig_t;

/* 外部变量 */
extern BodeConfig_t bode_config;
extern BodeData_t bode_buffer[BODE_MAX_SAMPLES];
extern uint16_t bode_buffer_idx;

/* 函数声明 */
void Bode_Init(void);
void Bode_Start(float freq, float pwm_amp, float pwm_bias, uint16_t samples);
void Bode_Stop(void);
void Bode_Process(void);
float Bode_GetPWM(void);
void Bode_SendAllData(void);

#ifdef __cplusplus
}
#endif

#endif /* __BODE_COLLECTOR_H */
