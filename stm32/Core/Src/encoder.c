/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   Encoder interface module for 4 mecanum wheels
  * @note    Uses TIM2, TIM3, TIM4, TIM5 in encoder mode
  ******************************************************************************
  */

#include "encoder.h"

/* Private defines -----------------------------------------------------------*/
#define ENCODER_COUNT_MAX   32767   /* Maximum count before overflow */
#define ENCODER_COUNT_MIN   -32768  /* Minimum count */

/* Private variables ---------------------------------------------------------*/
static uint8_t encoder_initialized = 0;

/* Static timer handles (persistent) */
static TIM_HandleTypeDef htim2_encoder;
static TIM_HandleTypeDef htim3_encoder;
static TIM_HandleTypeDef htim4_encoder;
static TIM_HandleTypeDef htim5_encoder;

/**
  * @brief  Configure GPIO for encoder input
  * @param  GPIOx: GPIO port
  * @param  GPIO_Pin: GPIO pin
  * @param  Alternate: GPIO alternate function
  */
static void Encoder_GPIO_Config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t Alternate)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = Alternate;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief  Configure timer as encoder
  * @param  htim: Timer handle
  * @param  TIMx: Timer instance
  * @retval HAL status
  */
static HAL_StatusTypeDef Encoder_TIM_Config(TIM_HandleTypeDef* htim, TIM_TypeDef* TIMx)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    
    htim->Instance = TIMx;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = ENCODER_TIM_PERIOD;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0x0F;  /* Add noise filter */
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0x0F;  /* Add noise filter */
    
    HAL_StatusTypeDef status = HAL_TIM_Encoder_Init(htim, &sConfig);
    if (status == HAL_OK) {
        status = HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    }
    
    return status;
}

/**
  * @brief  Initialize TIM2 as encoder (Motor A)
  * @retval 0: success, negative: error
  * @note   Pins: PA15 (CH1), PB3 (CH2)
  */
int Encoder_Init_TIM2(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    Encoder_GPIO_Config(GPIOA, GPIO_PIN_15, GPIO_AF1_TIM2);
    Encoder_GPIO_Config(GPIOB, GPIO_PIN_3, GPIO_AF1_TIM2);
    
    if (Encoder_TIM_Config(&htim2_encoder, TIM2) != HAL_OK) {
        return -1;
    }
    
    __HAL_TIM_SET_COUNTER(&htim2_encoder, 0);
    return 0;
}

/**
  * @brief  Initialize TIM3 as encoder (Motor B)
  * @retval 0: success, negative: error
  * @note   Pins: PB4 (CH1), PB5 (CH2)
  */
int Encoder_Init_TIM3(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    Encoder_GPIO_Config(GPIOB, GPIO_PIN_4, GPIO_AF2_TIM3);
    Encoder_GPIO_Config(GPIOB, GPIO_PIN_5, GPIO_AF2_TIM3);
    
    if (Encoder_TIM_Config(&htim3_encoder, TIM3) != HAL_OK) {
        return -1;
    }
    
    __HAL_TIM_SET_COUNTER(&htim3_encoder, 0);
    return 0;
}

/**
  * @brief  Initialize TIM4 as encoder (Motor C)
  * @retval 0: success, negative: error
  * @note   Pins: PB6 (CH1), PB7 (CH2)
  */
int Encoder_Init_TIM4(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    Encoder_GPIO_Config(GPIOB, GPIO_PIN_6, GPIO_AF2_TIM4);
    Encoder_GPIO_Config(GPIOB, GPIO_PIN_7, GPIO_AF2_TIM4);
    
    if (Encoder_TIM_Config(&htim4_encoder, TIM4) != HAL_OK) {
        return -1;
    }
    
    __HAL_TIM_SET_COUNTER(&htim4_encoder, 0);
    return 0;
}

/**
  * @brief  Initialize TIM5 as encoder (Motor D)
  * @retval 0: success, negative: error
  * @note   Pins: PA0 (CH1), PA1 (CH2)
  */
int Encoder_Init_TIM5(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    Encoder_GPIO_Config(GPIOA, GPIO_PIN_0, GPIO_AF2_TIM5);
    Encoder_GPIO_Config(GPIOA, GPIO_PIN_1, GPIO_AF2_TIM5);
    
    if (Encoder_TIM_Config(&htim5_encoder, TIM5) != HAL_OK) {
        return -1;
    }
    
    __HAL_TIM_SET_COUNTER(&htim5_encoder, 0);
    return 0;
}

/**
  * @brief  Initialize all encoders
  * @retval 0: success, negative: error code
  */
int Encoder_Init_All(void)
{
    int ret = 0;
    
    ret |= Encoder_Init_TIM2();
    ret |= Encoder_Init_TIM3();
    ret |= Encoder_Init_TIM4();
    ret |= Encoder_Init_TIM5();
    
    if (ret == 0) {
        encoder_initialized = 1;
    }
    
    return ret;
}

/**
  * @brief  Read encoder count and reset
  * @param  tim_num: Timer number (2, 3, 4, 5)
  * @retval Encoder count value (signed 16-bit)
  * @note   Reads and resets atomically. Call from single context only.
  */
int16_t Read_Encoder(uint8_t tim_num)
{
    int16_t encoder_val = 0;
    uint32_t primask;
    
    /* Disable interrupts for atomic read+clear */
    primask = __get_PRIMASK();
    __disable_irq();
    
    switch(tim_num)
    {
        case 2:
            encoder_val = (int16_t)htim2_encoder.Instance->CNT;
            htim2_encoder.Instance->CNT = 0;
            break;
        case 3:
            encoder_val = (int16_t)htim3_encoder.Instance->CNT;
            htim3_encoder.Instance->CNT = 0;
            break;
        case 4:
            encoder_val = (int16_t)htim4_encoder.Instance->CNT;
            htim4_encoder.Instance->CNT = 0;
            break;
        case 5:
            encoder_val = (int16_t)htim5_encoder.Instance->CNT;
            htim5_encoder.Instance->CNT = 0;
            break;
        default:
            encoder_val = 0;
            break;
    }
    
    /* Restore interrupts */
    __set_PRIMASK(primask);
    
    return encoder_val;
}

/**
  * @brief  Get encoder initialization status
  * @retval 1: initialized, 0: not initialized
  */
uint8_t Encoder_IsInitialized(void)
{
    return encoder_initialized;
}
