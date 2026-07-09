#ifndef __I2C_H
#define __I2C_H

#include "main.h"
#include <stdbool.h>

/* I2C pin definitions (Software I2C) */
#define I2C_SCL_PORT    GPIOB
#define I2C_SCL_PIN     GPIO_PIN_10
#define I2C_SDA_PORT    GPIOB
#define I2C_SDA_PIN     GPIO_PIN_11

/* I2C bit operations */
#define I2C_SCL_HIGH()  HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_LOW()   HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define I2C_SDA_HIGH()  HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_LOW()   HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define I2C_SDA_READ()  HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN)

/* I2C direction */
#define I2C_Direction_Transmitter   0x00
#define I2C_Direction_Receiver      0x01

/* I2C ACK/NACK */
#define I2C_ACK     0
#define I2C_NACK    1

/* Function prototypes */
void I2C_GPIOInit(void);
void I2C_Start(void);
void I2C_Stop(void);
bool I2C_WaitForAck(void);
void I2C_Ack(void);
void I2C_NAck(void);
void I2C_WriteByte(uint8_t Data);
uint8_t I2C_ReadByte(uint8_t Ack);

uint8_t I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);
uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr);
bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data);
bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data);
bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);

#endif /* __I2C_H */
