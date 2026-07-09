/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Motor PWM control module for 4 mecanum wheels
  * @note    Uses TIM1, TIM9, TIM10, TIM11 for 8 PWM channels (2 per motor)
  ******************************************************************************
  */

#include "motor.h"
#include "tim.h"

/* Private defines -----------------------------------------------------------*/
#define MOTOR_PWM_MAX   16700   /* Maximum PWM value (matches timer period) */
#define MOTOR_PWM_MIN   (-16700) /* Minimum PWM value */

/* Private variables ---------------------------------------------------------*/
static uint8_t motor_initialized = 0;

/**
  * @brief  Motor PWM initialization
  * @retval 0: success, -1: already initialized
  * @note   Must be called after MX_TIM1/9/10/11_Init()
  */
int Motor_Init(void)
{
    if (motor_initialized) {
        return -1;  /* Already initialized */
    }
    
    /* Start PWM channels */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    
    /* Enable pin initialization (PD3) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    /* Set all PWM to 0 (safe state) */
    Set_Pwm(0, 0, 0, 0);
    
    motor_initialized = 1;
    return 0;
}

/**
  * @brief  Check if motor enable pin is active
  * @retval 1: enabled, 0: disabled
  */
uint8_t Motor_IsEnabled(void)
{
    return (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET) ? 1 : 0;
}

/**
  * @brief  Clamp integer value to range
  * @param  value: Input value
  * @param  min: Minimum value
  * @param  max: Maximum value
  * @retval Clamped value
  */
static int clamp_int(int value, int min, int max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
  * @brief  Set PWM values for 4 motors
  * @param  motor_a: Motor A PWM value (will be clamped to +/-PWM_MAX)
  * @param  motor_b: Motor B PWM value
  * @param  motor_c: Motor C PWM value
  * @param  motor_d: Motor D PWM value
  * @retval None
  * @note   Uses differential PWM control for bidirectional motors
  *         Positive value = forward, Negative = reverse
  */
void Set_Pwm(int motor_a, int motor_b, int motor_c, int motor_d)
{
    /* Clamp inputs to safe range */
    motor_a = clamp_int(motor_a, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    motor_b = clamp_int(motor_b, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    motor_c = clamp_int(motor_c, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    motor_d = clamp_int(motor_d, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    
    /* Motor A - Differential PWM */
    if (motor_a < 0) {
        PWMA1 = (uint32_t)MOTOR_PWM_MAX;
        PWMA2 = (uint32_t)(MOTOR_PWM_MAX + motor_a);
    } else {
        PWMA2 = (uint32_t)MOTOR_PWM_MAX;
        PWMA1 = (uint32_t)(MOTOR_PWM_MAX - motor_a);
    }
    
    /* Motor B - Differential PWM */
    if (motor_b < 0) {
        PWMB1 = (uint32_t)MOTOR_PWM_MAX;
        PWMB2 = (uint32_t)(MOTOR_PWM_MAX + motor_b);
    } else {
        PWMB2 = (uint32_t)MOTOR_PWM_MAX;
        PWMB1 = (uint32_t)(MOTOR_PWM_MAX - motor_b);
    }
    
    /* Motor C - Differential PWM */
    if (motor_c < 0) {
        PWMC1 = (uint32_t)MOTOR_PWM_MAX;
        PWMC2 = (uint32_t)(MOTOR_PWM_MAX + motor_c);
    } else {
        PWMC2 = (uint32_t)MOTOR_PWM_MAX;
        PWMC1 = (uint32_t)(MOTOR_PWM_MAX - motor_c);
    }
    
    /* Motor D - Differential PWM */
    if (motor_d < 0) {
        PWMD1 = (uint32_t)MOTOR_PWM_MAX;
        PWMD2 = (uint32_t)(MOTOR_PWM_MAX + motor_d);
    } else {
        PWMD2 = (uint32_t)MOTOR_PWM_MAX;
        PWMD1 = (uint32_t)(MOTOR_PWM_MAX - motor_d);
    }
}

/**
  * @brief  Emergency stop - set all motors to 0
  * @retval None
  */
void Motor_EmergencyStop(void)
{
    PWMA1 = 0; PWMA2 = 0;
    PWMB1 = 0; PWMB2 = 0;
    PWMC1 = 0; PWMC2 = 0;
    PWMD1 = 0; PWMD2 = 0;
}

/**
  * @brief  Get motor initialization status
  * @retval 1: initialized, 0: not initialized
  */
uint8_t Motor_IsInitialized(void)
{
    return motor_initialized;
}
