/**
  ******************************************************************************
  * @file    oled.c
  * @brief   OLED显示模块 - 0.96寸128x64 SSD1306
  * @note    参考WHEELTEC R550源码
  ******************************************************************************
  */

#include "oled.h"
#include "oledfont.h"
#include "mecanum.h"
#include "pid.h"
#include "imu.h"
#include "battery.h"
#include <string.h>

/* GRAM缓冲区 */
uint8_t OLED_GRAM[128][8];

/**
  * @brief  OLED GPIO初始化
  */
static void OLED_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = OLED_RST_PIN | OLED_DC_PIN | OLED_SCLK_PIN | OLED_SDIN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
  * @brief  写字节到OLED
  * @param  dat: 数据
  * @param  cmd: OLED_CMD=命令, OLED_DATA=数据
  */
void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
    uint8_t i;
    
    if(cmd)
        OLED_DC_Set();
    else
        OLED_DC_Clr();
    
    for(i = 0; i < 8; i++)
    {
        OLED_SCLK_Clr();
        if(dat & 0x80)
            OLED_SDIN_Set();
        else
            OLED_SDIN_Clr();
        OLED_SCLK_Set();
        dat <<= 1;
    }
    
    OLED_DC_Set();
}

/**
  * @brief  设置显示位置
  * @param  x: X坐标 (0-127)
  * @param  y: Y坐标 (0-7)
  */
void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xB0 + y, OLED_CMD);
    OLED_WR_Byte(((x & 0xF0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte((x & 0x0F) | 0x01, OLED_CMD);
}

/**
  * @brief  开启OLED显示
  */
void OLED_Display_On(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0xAF, OLED_CMD);
}

/**
  * @brief  关闭OLED显示
  */
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0x8D, OLED_CMD);
    OLED_WR_Byte(0x10, OLED_CMD);
    OLED_WR_Byte(0xAE, OLED_CMD);
}

/**
  * @brief  清屏
  */
void OLED_Clear(void)
{
    uint8_t i, n;
    for(i = 0; i < 8; i++)
    {
        for(n = 0; n < 128; n++)
        {
            OLED_GRAM[n][i] = 0;
        }
    }
    OLED_Refresh_Gram();
}

/**
  * @brief  刷新GRAM到OLED
  */
void OLED_Refresh_Gram(void)
{
    uint8_t i, n;
    for(i = 0; i < 8; i++)
    {
        OLED_WR_Byte(0xB0 + i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for(n = 0; n < 128; n++)
        {
            OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA);
        }
    }
}

/**
  * @brief  画点
  * @param  x: X坐标 (0-127)
  * @param  y: Y坐标 (0-63)
  * @param  t: 1=填充, 0=清空
  */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t pos, bx, temp = 0;
    
    if(x > 127 || y > 63) return;
    
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    
    if(t)
        OLED_GRAM[x][pos] |= temp;
    else
        OLED_GRAM[x][pos] &= ~temp;
}

/**
  * @brief  显示字符
  * @param  x: X坐标
  * @param  y: Y坐标
  * @param  chr: 字符
  * @param  size: 字体大小 (12/16/24)
  */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size)
{
    uint8_t c = 0, i = 0;
    
    c = chr - ' ';
    
    if(x > 128 - 1) { x = 0; y = y + 2; }
    
    if(size == 16)
    {
        OLED_Set_Pos(x, y);
        for(i = 0; i < 8; i++)
            OLED_WR_Byte(F8X16[c * 16 + i], OLED_DATA);
        OLED_Set_Pos(x, y + 1);
        for(i = 0; i < 8; i++)
            OLED_WR_Byte(F8X16[c * 16 + i + 8], OLED_DATA);
    }
    else
    {
        OLED_Set_Pos(x, y);
        for(i = 0; i < 6; i++)
            OLED_WR_Byte(F6x8[c][i], OLED_DATA);
    }
}

/**
  * @brief  显示数字
  * @param  x: X坐标
  * @param  y: Y坐标
  * @param  num: 数字
  * @param  len: 长度
  * @param  size: 字体大小
  */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    
    for(t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if(enshow == 0 && t < (len - 1))
        {
            if(temp == 0)
            {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size);
    }
}

/**
  * @brief  显示字符串
  * @param  x: X坐标
  * @param  y: Y坐标
  * @param  str: 字符串
  */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
    uint8_t j = 0;
    while(str[j] != '\0')
    {
        OLED_ShowChar(x, y, str[j], 12);
        x += 6;
        if(x > 122) { x = 0; y += 2; }
        j++;
    }
}

/**
  * @brief  显示浮点数
  * @param  x: X坐标
  * @param  y: Y坐标
  * @param  num: 浮点数
  * @param  int_len: 整数部分长度
  * @param  dec_len: 小数部分长度
  */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len)
{
    char buf[16];
    int int_part;
    float frac_part;
    
    if(num < 0)
    {
        OLED_ShowChar(x, y, '-', 12);
        x += 6;
        num = -num;
    }
    
    int_part = (int)num;
    frac_part = num - int_part;
    
    OLED_ShowNum(x, y, int_part, int_len, 12);
    x += int_len * 6;
    
    OLED_ShowChar(x, y, '.', 12);
    x += 6;
    
    for(int i = 0; i < dec_len; i++)
    {
        frac_part *= 10;
        OLED_ShowChar(x, y, '0' + (int)frac_part % 10, 12);
        x += 6;
    }
}

/**
  * @brief  画线
  * @param  x1, y1: 起点坐标
  * @param  x2, y2: 终点坐标
  */
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    int16_t dx, dy, err, e2, sx, sy;
    
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    sx = (x1 < x2) ? 1 : -1;
    sy = (y1 < y2) ? 1 : -1;
    err = dx - dy;
    
    while(1)
    {
        OLED_DrawPoint(x1, y1, 1);
        
        if(x1 == x2 && y1 == y2) break;
        
        e2 = 2 * err;
        if(e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        if(e2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }
}

/**
  * @brief  画矩形
  * @param  x1, y1: 左上角坐标
  * @param  x2, y2: 右下角坐标
  */
void OLED_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    OLED_DrawLine(x1, y1, x2, y1);
    OLED_DrawLine(x1, y1, x1, y2);
    OLED_DrawLine(x2, y1, x2, y2);
    OLED_DrawLine(x1, y2, x2, y2);
}

/**
  * @brief  画圆
  * @param  x, y: 圆心坐标
  * @param  r: 半径
  */
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int16_t a = 0, b = r;
    int16_t d = 3 - 2 * r;
    
    while(a <= b)
    {
        OLED_DrawPoint(x + a, y + b, 1);
        OLED_DrawPoint(x + b, y + a, 1);
        OLED_DrawPoint(x - a, y + b, 1);
        OLED_DrawPoint(x - b, y + a, 1);
        OLED_DrawPoint(x + a, y - b, 1);
        OLED_DrawPoint(x + b, y - a, 1);
        OLED_DrawPoint(x - a, y - b, 1);
        OLED_DrawPoint(x - b, y - a, 1);
        
        if(d < 0)
            d += 4 * a + 6;
        else
        {
            d += 4 * (a - b) + 10;
            b--;
        }
        a++;
    }
}

/**
  * @brief  显示中文字符（需要字模数据）
  * @param  x: X坐标
  * @param  y: Y坐标
  * @param  index: 字符索引
  */
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t index)
{
    uint8_t t, adder = 0;
    
    OLED_Set_Pos(x, y);
    for(t = 0; t < 16; t++)
    {
        OLED_WR_Byte(Hzk[2 * index][t], OLED_DATA);
        adder += 1;
    }
    OLED_Set_Pos(x, y + 1);
    for(t = 0; t < 16; t++)
    {
        OLED_WR_Byte(Hzk[2 * index + 1][t], OLED_DATA);
        adder += 1;
    }
}

/**
  * @brief  初始化OLED
  */
void OLED_Init(void)
{
    OLED_GPIO_Init();
    
    OLED_RST_Clr();
    HAL_Delay(100);
    OLED_RST_Set();
    
    OLED_WR_Byte(0xAE, OLED_CMD); /* 关闭显示 */
    OLED_WR_Byte(0xD5, OLED_CMD); /* 设置时钟分频 */
    OLED_WR_Byte(80, OLED_CMD);
    OLED_WR_Byte(0xA8, OLED_CMD); /* 设置复用率 */
    OLED_WR_Byte(0x3F, OLED_CMD);
    OLED_WR_Byte(0xD3, OLED_CMD); /* 设置显示偏移 */
    OLED_WR_Byte(0x00, OLED_CMD);
    OLED_WR_Byte(0x40, OLED_CMD); /* 设置起始行 */
    OLED_WR_Byte(0x8D, OLED_CMD); /* 充电泵 */
    OLED_WR_Byte(0x14, OLED_CMD);
    OLED_WR_Byte(0x20, OLED_CMD); /* 内存寻址模式 */
    OLED_WR_Byte(0x02, OLED_CMD);
    OLED_WR_Byte(0xA1, OLED_CMD); /* 段重映射 */
    OLED_WR_Byte(0xC8, OLED_CMD); /* COM扫描方向 */
    OLED_WR_Byte(0xDA, OLED_CMD); /* COM硬件配置 */
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0x81, OLED_CMD); /* 对比度 */
    OLED_WR_Byte(0xCF, OLED_CMD);
    OLED_WR_Byte(0xD9, OLED_CMD); /* 预充电周期 */
    OLED_WR_Byte(0xF1, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); /* VCOMH取消水平 */
    OLED_WR_Byte(0x30, OLED_CMD);
    OLED_WR_Byte(0xA4, OLED_CMD); /* 全显示开 */
    OLED_WR_Byte(0xA6, OLED_CMD); /* 正常显示 */
    OLED_WR_Byte(0xAF, OLED_CMD); /* 开启显示 */
    
    OLED_Clear();
}

/**
  * @brief  更新OLED显示内容（四轮状态）
  * @note   显示格式：
  *         Line 1: Mecanum  GZ+1234
  *         Line 2: A+0.50  +0.48
  *         Line 3: B+0.50  +0.48
  *         Line 4: C+0.50  +0.48
  *         Line 5: D+0.50  +0.48
  *         Line 6: EN:ON  12.0V
  */
void OLED_UpdateStatus(void)
{
    char buf[32];
    
    /* 清除GRAM */
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    
    /* Line 0: 标题 */
    OLED_ShowString(0, 0, "Mecanum");
    OLED_ShowString(60, 0, "GZ");
    if(imu.gyro.z < 0)
    {
        OLED_ShowChar(78, 0, '-', 12);
        OLED_ShowNum(84, 0, -imu.gyro.z, 5, 12);
    }
    else
    {
        OLED_ShowChar(78, 0, '+', 12);
        OLED_ShowNum(84, 0, imu.gyro.z, 5, 12);
    }
    
    /* Line 1: 电机A */
    OLED_ShowString(0, 10, "A");
    if(Target_A < 0)
    {
        OLED_ShowChar(15, 10, '-', 12);
        OLED_ShowNum(21, 10, (int)(-Target_A * 1000), 5, 12);
    }
    else
    {
        OLED_ShowChar(15, 10, '+', 12);
        OLED_ShowNum(21, 10, (int)(Target_A * 1000), 5, 12);
    }
    if(Encoder_A < 0)
    {
        OLED_ShowChar(60, 10, '-', 12);
        OLED_ShowNum(66, 10, (int)(-Encoder_A * 1000), 5, 12);
    }
    else
    {
        OLED_ShowChar(60, 10, '+', 12);
        OLED_ShowNum(66, 10, (int)(Encoder_A * 1000), 5, 12);
    }
    
    /* Line 2: 电机B */
    OLED_ShowString(0, 20, "B");
    if(Target_B < 0)
    {
        OLED_ShowChar(15, 20, '-', 12);
        OLED_ShowNum(21, 20, (int)(-Target_B * 1000), 5, 12);
    }
    else
    {
        OLED_ShowChar(15, 20, '+', 12);
        OLED_ShowNum(21, 20, (int)(Target_B * 1000), 5, 12);
    }
    if(Encoder_B < 0)
    {
        OLED_ShowChar(60, 20, '-', 12);
        OLED_ShowNum(66, 20, (int)(-Encoder_B * 1000), 5, 12);
    }
    else
    {
        OLED_ShowChar(60, 20, '+', 12);
        OLED_ShowNum(66, 20, (int)(Encoder_B * 1000), 5, 12);
    }
    
    /* Line 3: 电机C */
    OLED_ShowString(0, 30, "C");
    if(Target_C < 0)
    {
        OLED_ShowChar(15, 30, '-', 12);
        OLED_ShowNum(21, 30, (int)(-Target_C * 1000), 5, 12);
    }
    else
    {
        OLED_ShowChar(15, 30, '+', 12);
        OLED_ShowNum(21, 30, (int)(Target_C * 1000), 5, 12);
    }
    if(Encoder_C < 0)
    {
        OLED_ShowChar(60, 30, '-', 12);
        OLED_ShowNum(66, 30, (int)(-Encoder_C * 1000), 5, 12);
    }
    else
    {
        OLED_ShowChar(60, 30, '+', 12);
        OLED_ShowNum(66, 30, (int)(Encoder_C * 1000), 5, 12);
    }
    
    /* Line 4: 电机D */
    OLED_ShowString(0, 40, "D");
    if(Target_D < 0)
    {
        OLED_ShowChar(15, 40, '-', 12);
        OLED_ShowNum(21, 40, (int)(-Target_D * 1000), 5, 12);
    }
    else
    {
        OLED_ShowChar(15, 40, '+', 12);
        OLED_ShowNum(21, 40, (int)(Target_D * 1000), 5, 12);
    }
    if(Encoder_D < 0)
    {
        OLED_ShowChar(60, 40, '-', 12);
        OLED_ShowNum(66, 40, (int)(-Encoder_D * 1000), 5, 12);
    }
    else
    {
        OLED_ShowChar(60, 40, '+', 12);
        OLED_ShowNum(66, 40, (int)(Encoder_D * 1000), 5, 12);
    }
    
    /* Line 5: 状态 */
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET)
        OLED_ShowString(0, 50, "EN:ON ");
    else
        OLED_ShowString(0, 50, "EN:OFF");
    
    /* 电池电压 */
    OLED_ShowNum(50, 50, (int)Voltage, 2, 12);
    OLED_ShowChar(62, 50, '.', 12);
    OLED_ShowNum(68, 50, (int)(Voltage * 100) % 100, 2, 12);
    OLED_ShowString(80, 50, "V");
    
    /* 刷新显示 */
    OLED_Refresh_Gram();
}
