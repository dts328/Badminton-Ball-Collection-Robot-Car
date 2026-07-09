#ifndef __BATTERY_H
#define __BATTERY_H

#include "main.h"

/* Battery voltage threshold */
#define BATTERY_LOW_VOLTAGE  11.0f   /* Low voltage threshold (V) */

/* External variables */
extern float Voltage;

/* Function prototypes */
void Battery_Init(void);
float Battery_GetVoltage(void);
uint8_t Battery_IsLow(void);

#endif /* __BATTERY_H */
