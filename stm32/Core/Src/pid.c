/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Incremental PID controller module
  ******************************************************************************
  */

#include "pid.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define PWM_MAX             16700.0f
#define PWM_MIN             (-16700.0f)
#define KP_MAX              10000.0f
#define KI_MAX              10000.0f
#define INTEGRAL_MAX        5000.0f   /* Anti-windup limit */
#define ERROR_DEADZONE      0.02f     /* Error deadzone (m/s) */

/* Motor parameter structures */
Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;

/* Global PID parameters - 基于FOPDT开环辨识结果 */
/* FOPDT模型: K=0.000083 (m/s/PWM), τ=71.3ms, L=113.8ms */
/* 拟合质量: 96.3% */
/* Z-N计算: Kp=6786, Ki=17902, Kd=77 */
/* 离散化补偿: Ki = Ki_continuous * dt = 17902 * 0.01 = 179 */
/* 去掉D项，避免碳刷噪声放大 */
float Velocity_KP = 6786.0f;
float Velocity_KI = 179.0f;    /* 已补偿dt=10ms */
float Velocity_KD = 0.0f;      /* 有刷电机不用D项 */

/**
  * @brief  Check if float is valid (not NaN or Inf)
  */
static uint8_t is_valid_float(float value)
{
    return !isnan(value) && !isinf(value);
}

/**
  * @brief  Initialize single PID controller
  */
static void PID_InitController(Motor_parameter *motor)
{
    motor->Encoder = 0.0f;
    motor->Motor_Pwm = 0.0f;
    motor->Target = 0.0f;
    motor->Velocity_KP = Velocity_KP;
    motor->Velocity_KI = Velocity_KI;
}

/**
  * @brief  Initialize all PID controllers
  */
void PID_Init(void)
{
    PID_InitController(&MOTOR_A);
    PID_InitController(&MOTOR_B);
    PID_InitController(&MOTOR_C);
    PID_InitController(&MOTOR_D);
}

/**
  * @brief  Generic incremental PI controller
  * @param  motor: Pointer to motor parameter structure
  * @param  encoder: Measured encoder speed
  * @param  target: Target speed
  * @retval PWM output value
  */
/* File-level static arrays (accessible by PID_Reset) */
static float pid_last_bias[4] = {0};
static float pid_pwm[4] = {0};

static int Incremental_PI(Motor_parameter *motor, float encoder, float target)
{
    float bias, p_term, i_term, delta;
    
    /* Determine motor index */
    int idx;
    if (motor == &MOTOR_A) idx = 0;
    else if (motor == &MOTOR_B) idx = 1;
    else if (motor == &MOTOR_C) idx = 2;
    else idx = 3;
    
    /* Calculate deviation */
    bias = target - encoder;
    
    /* Calculate P and I terms (use real error) */
    p_term = motor->Velocity_KP * (bias - pid_last_bias[idx]);
    i_term = motor->Velocity_KI * bias;
    
    /* --- Fix: Incremental deadzone anti-lock --- */
    if (fabsf(bias) < ERROR_DEADZONE) {
        i_term = 0.0f;  /* 1. Cut off integral accumulation */
        
        if (target == 0.0f) {
            /* 2. Leak brake: if target is 0 and speed is tiny, decay PWM */
            pid_pwm[idx] = pid_pwm[idx] * 0.8f;  /* Decay 20% per 5ms (control cycle) */
        }
    }
    
    /* Calculate delta */
    delta = p_term + i_term;
    
    /* Validate delta */
    if (!is_valid_float(delta)) {
        delta = 0.0f;
    }
    
    /* Calculate predicted next PWM */
    float next_pwm = pid_pwm[idx] + delta;
    
    /* Anti-windup: clamp PWM output */
    if (next_pwm > PWM_MAX) {
        next_pwm = PWM_MAX;
    } else if (next_pwm < PWM_MIN) {
        next_pwm = PWM_MIN;
    }
    
    /* Update output and history */
    pid_pwm[idx] = next_pwm;
    pid_last_bias[idx] = bias;
    
    return (int)pid_pwm[idx];
}

/**
  * @brief  Incremental PI controller for Motor A
  */
int Incremental_PI_A(float Encoder, float Target)
{
    return Incremental_PI(&MOTOR_A, Encoder, Target);
}

/**
  * @brief  Incremental PI controller for Motor B
  */
int Incremental_PI_B(float Encoder, float Target)
{
    return Incremental_PI(&MOTOR_B, Encoder, Target);
}

/**
  * @brief  Incremental PI controller for Motor C
  */
int Incremental_PI_C(float Encoder, float Target)
{
    return Incremental_PI(&MOTOR_C, Encoder, Target);
}

/**
  * @brief  Incremental PI controller for Motor D
  */
int Incremental_PI_D(float Encoder, float Target)
{
    return Incremental_PI(&MOTOR_D, Encoder, Target);
}

/**
  * @brief  Set PID parameters
  * @retval 0: success, -1: invalid parameters
  */
int PID_SetParam(float kp, float ki)
{
    /* Validate parameters */
    if (!is_valid_float(kp) || !is_valid_float(ki)) {
        return -1;
    }
    
    if (kp < 0 || kp > KP_MAX || ki < 0 || ki > KI_MAX) {
        return -1;
    }
    
    Velocity_KP = kp;
    Velocity_KI = ki;
    Velocity_KD = 0.0f;  /* 电机速度环通常不需要微分 */
    
    /* Update all controllers */
    MOTOR_A.Velocity_KP = kp;
    MOTOR_A.Velocity_KI = ki;
    MOTOR_B.Velocity_KP = kp;
    MOTOR_B.Velocity_KI = ki;
    MOTOR_C.Velocity_KP = kp;
    MOTOR_C.Velocity_KI = ki;
    MOTOR_D.Velocity_KP = kp;
    MOTOR_D.Velocity_KI = ki;
    
    return 0;
}

/**
  * @brief  Get PID parameters
  */
void PID_GetParam(float *kp, float *ki, float *kd)
{
    if (kp != NULL) *kp = Velocity_KP;
    if (ki != NULL) *ki = Velocity_KI;
    if (kd != NULL) *kd = Velocity_KD;
}

/**
  * @brief  Reset all PID controllers (including internal state)
  */
void PID_Reset(void)
{
    PID_Init();
    
    /* Clear internal static state */
    for(int i = 0; i < 4; i++) {
        pid_last_bias[i] = 0.0f;
        pid_pwm[i] = 0.0f;
    }
}
