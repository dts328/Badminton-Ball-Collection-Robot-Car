/**
  ******************************************************************************
  * @file    control_source.c
  * @brief   Control source arbitration implementation
  ******************************************************************************
  */

#include "control_source.h"

/* Private variables */
static ControlSource_t control_source = CONTROL_SOURCE_UART;
static uint32_t ros_last_time = 0;

/**
  * @brief  Set control source
  * @param  source: CONTROL_SOURCE_UART or CONTROL_SOURCE_ROS
  */
void Control_SetSource(ControlSource_t source)
{
    control_source = source;
    
    /* Update ROS timestamp when switching to ROS */
    if(source == CONTROL_SOURCE_ROS)
    {
        ros_last_time = HAL_GetTick();
    }
}

/**
  * @brief  Get current control source
  * @retval Current control source
  */
ControlSource_t Control_GetSource(void)
{
    return control_source;
}

/**
  * @brief  Get last ROS command timestamp
  * @retval Timestamp in ms
  */
uint32_t Control_GetRosLastTime(void)
{
    return ros_last_time;
}

/**
  * @brief  Update ROS command timestamp
  */
void Control_UpdateRosTime(void)
{
    ros_last_time = HAL_GetTick();
}
