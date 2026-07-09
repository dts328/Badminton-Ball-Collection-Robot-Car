#include "imu.h"
#include "icm20948.h"

/* Global IMU data structure */
IMU_DATA_t imu;

/**
  * @brief  Initialize IMU
  * @param  None
  * @retval None
  */
void IMU_Init(void)
{
    /* Clear IMU data */
    imu.Deviation_accel.x = 0;
    imu.Deviation_accel.y = 0;
    imu.Deviation_accel.z = 0;
    imu.Deviation_gyro.x = 0;
    imu.Deviation_gyro.y = 0;
    imu.Deviation_gyro.z = 0;
    imu.Original_accel.x = 0;
    imu.Original_accel.y = 0;
    imu.Original_accel.z = 0;
    imu.Original_gyro.x = 0;
    imu.Original_gyro.y = 0;
    imu.Original_gyro.z = 0;
    imu.gyro.x = 0;
    imu.gyro.y = 0;
    imu.gyro.z = 0;
    imu.accel.x = 0;
    imu.accel.y = 0;
    imu.accel.z = 0;
    
    /* Initialize ICM20948 */
    ICM20948_Init();
}

/**
  * @brief  Set current IMU data as zero offset
  * @param  None
  * @retval None
  */
void IMU_SetZero(void)
{
    ImuData_Copy(&imu.Deviation_gyro, &imu.gyro);
    ImuData_Copy(&imu.Deviation_accel, &imu.accel);
}

/**
  * @brief  Get IMU data (accel + gyro)
  * @param  None
  * @retval None
  * @note   Call this function periodically (100Hz)
  */
void IMU_GetData(void)
{
    /* Get accelerometer data */
    ICM20948_GetAccel(&imu.accel.x, &imu.accel.y, &imu.accel.z);
    
    /* Get gyroscope data */
    ICM20948_GetGyro(&imu.gyro.x, &imu.gyro.y, &imu.gyro.z);
    
    /* Save original data */
    imu.Original_accel.x = imu.accel.x;
    imu.Original_accel.y = imu.accel.y;
    imu.Original_accel.z = imu.accel.z;
    imu.Original_gyro.x = imu.gyro.x;
    imu.Original_gyro.y = imu.gyro.y;
    imu.Original_gyro.z = imu.gyro.z;
    
    /* Remove zero drift */
    imu.accel.x = imu.Original_accel.x - imu.Deviation_accel.x;
    imu.accel.y = imu.Original_accel.y - imu.Deviation_accel.y;
    imu.accel.z = imu.Original_accel.z - imu.Deviation_accel.z + 16384;  /* Add 1g for Z-axis */
    imu.gyro.x = imu.Original_gyro.x - imu.Deviation_gyro.x;
    imu.gyro.y = imu.Original_gyro.y - imu.Deviation_gyro.y;
    imu.gyro.z = imu.Original_gyro.z - imu.Deviation_gyro.z;
}

/**
  * @brief  Copy IMU data
  * @param  req_val: Destination
  * @param  copied_val: Source
  * @retval None
  */
void ImuData_Copy(IMU_BASE_t *req_val, const IMU_BASE_t *copied_val)
{
    req_val->x = copied_val->x;
    req_val->y = copied_val->y;
    req_val->z = copied_val->z;
}
