/**
  ******************************************************************************
  * @file    battery.h
  * @brief   Battery voltage monitoring module header
  ******************************************************************************
  */

#ifndef __BATTERY_H
#define __BATTERY_H

#include "main.h"

/* Battery voltage threshold */
#define BATTERY_LOW_VOLTAGE  11.0f   /* Low voltage threshold (V) */
#define BATTERY_FULL_VOLTAGE 12.6f   /* Full charge voltage (V) */
#define BATTERY_EMPTY_VOLTAGE 9.0f   /* Empty voltage (V) */

/* External variables */
extern float Voltage;

/* Function prototypes */
void Battery_Init(void);
float Battery_GetVoltage(void);
uint8_t Battery_IsLow(void);
uint8_t Battery_GetPercentage(void);

#endif /* __BATTERY_H */
