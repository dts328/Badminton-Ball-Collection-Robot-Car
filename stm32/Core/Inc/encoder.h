/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   Encoder interface module header
  ******************************************************************************
  */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Exported defines ----------------------------------------------------------*/
#define ENCODER_TIM_PERIOD  65535   /* 16-bit timer period */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize all encoders
  * @retval 0: success, negative: error
  */
int Encoder_Init_All(void);

/**
  * @brief  Read encoder count and reset
  * @param  tim_num: Timer number (2, 3, 4, 5)
  * @retval Encoder count value (signed)
  * @note   Atomic read+clear operation
  */
int16_t Read_Encoder(uint8_t tim_num);

/**
  * @brief  Get encoder initialization status
  * @retval 1: initialized, 0: not initialized
  */
uint8_t Encoder_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
