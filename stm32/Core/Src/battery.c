/**
  ******************************************************************************
  * @file    battery.c
  * @brief   Battery voltage monitoring via ADC
  ******************************************************************************
  */

#include "battery.h"
#include "adc.h"
#include <string.h>

/* Global voltage variable */
float Voltage = 12.0f;

/* ADC buffer for averaging */
#define ADC_BUF_SIZE  16
static uint16_t adc_buffer[ADC_BUF_SIZE];
static uint8_t adc_index = 0;
static uint8_t adc_ready = 0;

/* Battery voltage divider: 10K/10K -> 1:2 ratio */
/* ADC reference: 3.3V, 12-bit resolution */
#define VOLTAGE_DIVIDER_RATIO  2.0f
#define ADC_REF_VOLTAGE        3.3f
#define ADC_RESOLUTION         4096.0f

/**
  * @brief  Battery initialization with ADC
  * @param  None
  * @retval None
  */
void Battery_Init(void)
{
    /* Clear ADC buffer */
    memset(adc_buffer, 0, sizeof(adc_buffer));
    adc_index = 0;
    adc_ready = 0;
    
    /* Start ADC DMA conversion */
    /* Note: ADC should be configured in CubeMX for channel connected to battery */
    #ifdef HAL_ADC_MODULE_ENABLED
    /* Start ADC with DMA if available */
    extern ADC_HandleTypeDef hadc1;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_SIZE);
    #endif
    
    /* Initial voltage reading */
    Voltage = 12.0f;
}

/**
  * @brief  ADC conversion complete callback
  * @param  hadc: ADC handle
  * @retval None
  */
#ifdef HAL_ADC_MODULE_ENABLED
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_ready = 1;
    }
}
#endif

/**
  * @brief  Calculate average ADC value
  * @param  None
  * @retval Average ADC value (0-4095)
  */
static uint16_t Battery_GetADCValue(void)
{
    uint32_t sum = 0;
    uint8_t i;
    
    /* Average all samples */
    for (i = 0; i < ADC_BUF_SIZE; i++)
    {
        sum += adc_buffer[i];
    }
    
    return (uint16_t)(sum / ADC_BUF_SIZE);
}

/**
  * @brief  Get battery voltage
  * @param  None
  * @retval Battery voltage in volts
  */
float Battery_GetVoltage(void)
{
    #ifdef HAL_ADC_MODULE_ENABLED
    uint16_t adc_value;
    float adc_voltage;
    
    /* Get averaged ADC value */
    adc_value = Battery_GetADCValue();
    
    /* Convert to voltage */
    adc_voltage = (float)adc_value * ADC_REF_VOLTAGE / ADC_RESOLUTION;
    
    /* Apply voltage divider ratio */
    Voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO;
    
    /* Sanity check */
    if (Voltage < 5.0f) Voltage = 5.0f;
    if (Voltage > 17.0f) Voltage = 17.0f;
    
    #else
    /* Fallback: return default if ADC not available */
    Voltage = 12.0f;
    #endif
    
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

/**
  * @brief  Get battery percentage (approximate)
  * @param  None
  * @retval Battery percentage (0-100)
  */
uint8_t Battery_GetPercentage(void)
{
    float voltage = Battery_GetVoltage();
    
    /* Li-Po 3S battery: 12.6V full, 9.0V empty */
    if (voltage >= 12.6f) return 100;
    if (voltage <= 9.0f) return 0;
    
    /* Linear approximation */
    return (uint8_t)((voltage - 9.0f) * 100.0f / 3.6f);
}
