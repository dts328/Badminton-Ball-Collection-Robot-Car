/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Motor PWM control module header
  ******************************************************************************
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Exported defines ----------------------------------------------------------*/
#define PWM_MAX         16700   /* Maximum PWM value */

/* Motor A control pins (TIM10, TIM11) */
#define PWMA1           TIM10->CCR1   /* PB8 */
#define PWMA2           TIM11->CCR1   /* PB9 */

/* Motor B control pins (TIM9) */
#define PWMB1           TIM9->CCR1    /* PE5 */
#define PWMB2           TIM9->CCR2    /* PE6 */

/* Motor C control pins (TIM1) */
#define PWMC1           TIM1->CCR2    /* PE11 */
#define PWMC2           TIM1->CCR1    /* PE9 */

/* Motor D control pins (TIM1) */
#define PWMD1           TIM1->CCR4    /* PE14 */
#define PWMD2           TIM1->CCR3    /* PE13 */

/* Enable pin (PD3, active low) - use HAL_GPIO_ReadPin instead of PDin macro */
#define EN_READ()       HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3)
#define EN_IS_ACTIVE()  (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET)

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize motor PWM hardware
  * @retval 0: success, -1: already initialized
  */
int Motor_Init(void);

/**
  * @brief  Check if motor enable pin is active
  * @retval 1: enabled, 0: disabled
  */
uint8_t Motor_IsEnabled(void);

/**
  * @brief  Set PWM values for all 4 motors
  * @param  motor_a: Motor A PWM (-PWM_MAX to +PWM_MAX)
  * @param  motor_b: Motor B PWM (-PWM_MAX to +PWM_MAX)
  * @param  motor_c: Motor C PWM (-PWM_MAX to +PWM_MAX)
  * @param  motor_d: Motor D PWM (-PWM_MAX to +PWM_MAX)
  * @note   Inputs are clamped to safe range
  */
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d);

/**
  * @brief  Emergency stop all motors
  */
void Motor_EmergencyStop(void);

/**
  * @brief  Get motor initialization status
  * @retval 1: initialized, 0: not initialized
  */
uint8_t Motor_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
