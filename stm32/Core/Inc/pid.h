/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Incremental PID controller module header
  ******************************************************************************
  */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Motor parameter structure
  */
typedef struct
{
    float Encoder;      /* Real-time speed from encoder (m/s) */
    float Motor_Pwm;    /* Motor PWM output value */
    float Target;       /* Target speed (m/s) */
    float Velocity_KP;  /* Speed control KP parameter */
    float Velocity_KI;  /* Speed control KI parameter */
} Motor_parameter;

/* Exported variables --------------------------------------------------------*/
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern float Velocity_KP;
extern float Velocity_KI;
extern float Velocity_KD;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize all PID controllers
  */
void PID_Init(void);

/**
  * @brief  Incremental PI controller for Motor A
  * @param  Encoder: Measured speed
  * @param  Target: Target speed
  * @retval PWM output
  */
int Incremental_PI_A(float Encoder, float Target);

/**
  * @brief  Incremental PI controller for Motor B
  */
int Incremental_PI_B(float Encoder, float Target);

/**
  * @brief  Incremental PI controller for Motor C
  */
int Incremental_PI_C(float Encoder, float Target);

/**
  * @brief  Incremental PI controller for Motor D
  */
int Incremental_PI_D(float Encoder, float Target);

/**
  * @brief  Set PID parameters
  * @param  kp: KP value (0-10000)
  * @param  ki: KI value (0-10000)
  * @retval 0: success, -1: invalid parameters
  */
int PID_SetParam(float kp, float ki);

/**
  * @brief  Get PID parameters
  * @param  kp: Pointer to store KP value
  * @param  ki: Pointer to store KI value
  * @param  kd: Pointer to store KD value
  */
void PID_GetParam(float *kp, float *ki, float *kd);

/**
  * @brief  Reset all PID controllers
  */
void PID_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
