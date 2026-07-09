#include "battery.h"

/* Global voltage variable */
float Voltage = 12.0f;  /* Default voltage */

/**
  * @brief  Battery initialization (simplified - no ADC)
  * @param  None
  * @retval None
  */
void Battery_Init(void)
{
    /* Default voltage - ADC not available */
    Voltage = 12.0f;
}

/**
  * @brief  Get battery voltage (simplified - returns default)
  * @param  None
  * @retval Battery voltage in volts
  */
float Battery_GetVoltage(void)
{
    /* Simplified: return default voltage */
    /* TODO: Implement actual voltage measurement if ADC available */
    return Voltage;
}

/**
  * @brief  Check if battery is low
  * @param  None
  * @retval 1: low voltage, 0: normal
  */
uint8_t Battery_IsLow(void)
{
    Voltage = Battery_GetVoltage();
    return (Voltage < BATTERY_LOW_VOLTAGE) ? 1 : 0;
}
