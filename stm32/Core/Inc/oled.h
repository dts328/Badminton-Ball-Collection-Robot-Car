/**
  ******************************************************************************
  * @file    oled.h
  * @brief   OLED显示模块 - 0.96寸128x64 SSD1306
  ******************************************************************************
  */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* OLED引脚定义 */
#define OLED_RST_PORT   GPIOD
#define OLED_RST_PIN    GPIO_PIN_12
#define OLED_DC_PORT    GPIOD
#define OLED_DC_PIN     GPIO_PIN_11
#define OLED_SCLK_PORT  GPIOD
#define OLED_SCLK_PIN   GPIO_PIN_14
#define OLED_SDIN_PORT  GPIOD
#define OLED_SDIN_PIN   GPIO_PIN_13

/* OLED引脚操作 */
#define OLED_RST_Clr()  HAL_GPIO_WritePin(OLED_RST_PORT, OLED_RST_PIN, GPIO_PIN_RESET)
#define OLED_RST_Set()  HAL_GPIO_WritePin(OLED_RST_PORT, OLED_RST_PIN, GPIO_PIN_SET)
#define OLED_DC_Clr()   HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_RESET)
#define OLED_DC_Set()   HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET)
#define OLED_SCLK_Clr() HAL_GPIO_WritePin(OLED_SCLK_PORT, OLED_SCLK_PIN, GPIO_PIN_RESET)
#define OLED_SCLK_Set() HAL_GPIO_WritePin(OLED_SCLK_PORT, OLED_SCLK_PIN, GPIO_PIN_SET)
#define OLED_SDIN_Clr() HAL_GPIO_WritePin(OLED_SDIN_PORT, OLED_SDIN_PIN, GPIO_PIN_RESET)
#define OLED_SDIN_Set() HAL_GPIO_WritePin(OLED_SDIN_PORT, OLED_SDIN_PIN, GPIO_PIN_SET)

#define OLED_CMD  0
#define OLED_DATA 1

#define OLED_Width  128
#define OLED_Height 64

/* 函数声明 */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len);
void OLED_Refresh_Gram(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t);
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void OLED_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t index);

/* 显示更新函数 */
void OLED_UpdateStatus(void);

/* GRAM缓冲区 */
extern uint8_t OLED_GRAM[128][8];

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
