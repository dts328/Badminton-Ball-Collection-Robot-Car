/**
  ******************************************************************************
  * @file    control_source.h
  * @brief   Control source arbitration interface
  ******************************************************************************
  */

#ifndef __CONTROL_SOURCE_H
#define __CONTROL_SOURCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Control source enum */
typedef enum {
    CONTROL_SOURCE_IDLE = 0,
    CONTROL_SOURCE_UART = 1,
    CONTROL_SOURCE_ROS  = 2,
} ControlSource_t;

/* Function prototypes */
void Control_SetSource(ControlSource_t source);
ControlSource_t Control_GetSource(void);
uint32_t Control_GetRosLastTime(void);
void Control_UpdateRosTime(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_SOURCE_H */
