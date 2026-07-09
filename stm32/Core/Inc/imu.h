#ifndef __IMU_H
#define __IMU_H

#include "main.h"

/* IMU base data structure */
typedef struct
{
    short x;
    short y;
    short z;
} IMU_BASE_t;

/* IMU data structure with offset */
typedef struct
{
    IMU_BASE_t Deviation_accel;  /* Accelerometer offset */
    IMU_BASE_t Deviation_gyro;   /* Gyroscope offset */
    IMU_BASE_t Original_accel;   /* Original accelerometer data */
    IMU_BASE_t Original_gyro;    /* Original gyroscope data */
    IMU_BASE_t gyro;             /* Processed gyroscope data */
    IMU_BASE_t accel;            /* Processed accelerometer data */
} IMU_DATA_t;

/* External variables */
extern IMU_DATA_t imu;

/* Function prototypes */
void IMU_Init(void);
void IMU_GetData(void);
void IMU_SetZero(void);
void ImuData_Copy(IMU_BASE_t *req_val, const IMU_BASE_t *copied_val);

#endif /* __IMU_H */
