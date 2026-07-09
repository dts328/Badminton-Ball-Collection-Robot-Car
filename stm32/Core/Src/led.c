#include "led.h"

/**
  * @brief  LED GPIO initialization
  * @param  None
  * @retval None
  */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOE_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
    
    LED_LOW();
}

/**
  * @brief  LED flash function
  * @param  time: Flash period in ms
  * @retval None
  */
void LED_Flash(uint16_t time)
{
    static uint16_t count = 0;
    
    count++;
    if(count >= time)
    {
        count = 0;
        LED_TOGGLE();
    }
}

/**
  * @brief  Buzzer GPIO initialization
  * @param  None
  * @retval None
  */
void Buzzer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);
    
    BUZZER_OFF();
}

/**
  * @brief  Buzzer beep function
  * @param  time_ms: Beep duration in ms
  * @retval None
  */
void Buzzer_Beep(uint16_t time_ms)
{
    BUZZER_ON();
    HAL_Delay(time_ms);
    BUZZER_OFF();
}
