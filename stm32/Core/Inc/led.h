#ifndef __LED_H
#define __LED_H

#include "main.h"

/* LED pin */
#define LED_PORT    GPIOE
#define LED_PIN     GPIO_PIN_8
#define LED_HIGH()  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define LED_LOW()   HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_TOGGLE() HAL_GPIO_TogglePin(LED_PORT, LED_PIN)

/* Buzzer pin */
#define BUZZER_PORT GPIOA
#define BUZZER_PIN  GPIO_PIN_8
#define BUZZER_ON() HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET)
#define BUZZER_OFF() HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET)

/* Function prototypes */
void LED_Init(void);
void LED_Flash(uint16_t time);
void Buzzer_Init(void);
void Buzzer_Beep(uint16_t time_ms);

#endif /* __LED_H */
