#include "icm20948.h"
#include "i2c.h"

/* Gyro offset for calibration */
static ICM20948_ST_SENSOR_DATA gstGyroOffset = {0, 0, 0};

/**
  * @brief  Calibrate gyroscope offset (32 samples average)
  * @param  None
  * @retval None
  */
static void ICM20948_GyroOffset(void)
{
    uint8_t i;
    int16_t s16Gx = 0, s16Gy = 0, s16Gz = 0;
    int32_t s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;
    
    for(i = 0; i < 32; i++)
    {
        ICM20948_GetGyro(&s16Gx, &s16Gy, &s16Gz);
        s32TempGx += s16Gx;
        s32TempGy += s16Gy;
        s32TempGz += s16Gz;
        HAL_Delay(10);
    }
    
    gstGyroOffset.s16X = s32TempGx >> 5;  /* Divide by 32 */
    gstGyroOffset.s16Y = s32TempGy >> 5;
    gstGyroOffset.s16Z = s32TempGz >> 5;
}

/**
  * @brief  Calculate average value with 8-point sliding filter
  * @param  pIndex: Pointer to current index
  * @param  pAvgBuffer: Pointer to average buffer
  * @param  InVal: Input value
  * @param  pOutVal: Pointer to output value
  * @retval None
  */
static void ICM20948_CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal)
{
    uint8_t i;
    
    *(pAvgBuffer + ((*pIndex)++)) = InVal;
    *pIndex &= 0x07;
    
    *pOutVal = 0;
    for(i = 0; i < 8; i++)
    {
        *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
}

/**
  * @brief  Read gyroscope raw data with filtering
  * @param  ps16X: Pointer to store X-axis value
  * @param  ps16Y: Pointer to store Y-axis value
  * @param  ps16Z: Pointer to store Z-axis value
  * @retval None
  */
static void ICM20948_GyroRead(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    uint8_t u8Buf[6];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    
    /* Read gyro data */
    u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_XOUT_L);
    u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_XOUT_H);
    s16Buf[0] = (u8Buf[1] << 8) | u8Buf[0];
    
    u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_YOUT_L);
    u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_YOUT_H);
    s16Buf[1] = (u8Buf[1] << 8) | u8Buf[0];
    
    u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_ZOUT_L);
    u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_ZOUT_H);
    s16Buf[2] = (u8Buf[1] << 8) | u8Buf[0];
    
    /* Apply 8-point sliding filter */
    for(i = 0; i < 3; i++)
    {
        ICM20948_CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    
    /* Subtract offset */
    *ps16X = s32OutBuf[0] - gstGyroOffset.s16X;
    *ps16Y = s32OutBuf[1] - gstGyroOffset.s16Y;
    *ps16Z = s32OutBuf[2] - gstGyroOffset.s16Z;
}

/**
  * @brief  Read accelerometer raw data with filtering
  * @param  ps16X: Pointer to store X-axis value
  * @param  ps16Y: Pointer to store Y-axis value
  * @param  ps16Z: Pointer to store Z-axis value
  * @retval None
  */
static void ICM20948_AccelRead(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    uint8_t u8Buf[2];
    int16_t s16Buf[3] = {0};
    uint8_t i;
    int32_t s32OutBuf[3] = {0};
    static ICM20948_ST_AVG_DATA sstAvgBuf[3];
    
    /* Read accel data */
    u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_XOUT_L);
    u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_XOUT_H);
    s16Buf[0] = (u8Buf[1] << 8) | u8Buf[0];
    
    u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_YOUT_L);
    u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_YOUT_H);
    s16Buf[1] = (u8Buf[1] << 8) | u8Buf[0];
    
    u8Buf[0] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_ZOUT_L);
    u8Buf[1] = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_ZOUT_H);
    s16Buf[2] = (u8Buf[1] << 8) | u8Buf[0];
    
    /* Apply 8-point sliding filter */
    for(i = 0; i < 3; i++)
    {
        ICM20948_CalAvgValue(&sstAvgBuf[i].u8Index, sstAvgBuf[i].s16AvgBuffer, s16Buf[i], s32OutBuf + i);
    }
    
    *ps16X = s32OutBuf[0];
    *ps16Y = s32OutBuf[1];
    *ps16Z = s32OutBuf[2];
}

/**
  * @brief  Check if ICM20948 is connected
  * @param  None
  * @retval true: connected, false: not connected
  */
static bool ICM20948_Check(void)
{
    if(REG_VAL_WIA == I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_WIA))
        return true;
    return false;
}

/**
  * @brief  Initialize ICM20948 hardware
  * @param  None
  * @retval None
  */
static void ICM20948_HwInit(void)
{
    /* User bank 0 */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1, REG_VAL_ALL_RGE_RESET);
    HAL_Delay(10);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1, REG_VAL_RUN_MODE);
    
    /* User bank 2 */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2);
    
    /* Gyro config: 500dps, DLPF */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_SMPLRT_DIV, 0x07);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_1, 
                     REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_500DPS | REG_VAL_BIT_GYRO_DLPF);
    
    /* Accel config: 2g, DLPF */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG, 
                     REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF);
    
    /* User bank 0 */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
    
    HAL_Delay(100);
}

/**
  * @brief  Read secondary I2C device (magnetometer)
  * @param  u8I2CAddr: I2C address
  * @param  u8RegAddr: Register address
  * @param  u8Len: Data length
  * @param  pu8data: Data buffer
  * @retval None
  */
static void ICM20948_ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
{
    uint8_t i;
    uint8_t u8Temp;
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, u8I2CAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG, u8RegAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN | u8Len);
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
    
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    HAL_Delay(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    
    for(i = 0; i < u8Len; i++)
    {
        *(pu8data + i) = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00 + i);
    }
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3);
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN) & (REG_VAL_BIT_MASK_LEN));
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, u8Temp);
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
}

/**
  * @brief  Write to secondary I2C device (magnetometer)
  * @param  u8I2CAddr: I2C address
  * @param  u8RegAddr: Register address
  * @param  u8data: Data to write
  * @retval None
  */
static void ICM20948_WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
{
    uint8_t u8Temp;
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_ADDR, u8I2CAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_REG, u8RegAddr);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_DO, u8data);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN | 1);
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
    
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    HAL_Delay(5);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp);
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3);
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN) & (REG_VAL_BIT_MASK_LEN));
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, u8Temp);
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0);
}

/**
  * @brief  Check magnetometer connection
  * @param  None
  * @retval true: connected, false: not connected
  */
static bool ICM20948_MagCheck(void)
{
    uint8_t u8Ret[2];
    
    ICM20948_ReadSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_READ,
                           REG_ADD_MAG_WIA1, 2, u8Ret);
    
    if((u8Ret[0] == REG_VAL_MAG_WIA1) && (u8Ret[1] == REG_VAL_MAG_WIA2))
        return true;
    
    return false;
}

/**
  * @brief  Initialize ICM20948
  * @param  None
  * @retval None
  */
void ICM20948_Init(void)
{
    /* Initialize I2C */
    I2C_GPIOInit();
    
    /* Check connection */
    if(ICM20948_Check())
    {
        /* Hardware initialization */
        ICM20948_HwInit();
        
        /* Gyro offset calibration */
        ICM20948_GyroOffset();
        
        /* Check and init magnetometer */
        ICM20948_MagCheck();
        ICM20948_WriteSecondary(I2C_ADD_ICM20948_AK09916 | I2C_ADD_ICM20948_AK09916_WRITE,
                                REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);
    }
}

/**
  * @brief  Get ICM20948 device ID
  * @param  None
  * @retval Device ID
  */
uint8_t ICM20948_GetDeviceID(void)
{
    return I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_WIA);
}

/**
  * @brief  Test ICM20948 connection
  * @param  None
  * @retval true: connected, false: not connected
  */
bool ICM20948_TestConnection(void)
{
    return ICM20948_Check();
}

/**
  * @brief  Get gyroscope data
  * @param  ps16X: Pointer to store X-axis value
  * @param  ps16Y: Pointer to store Y-axis value
  * @param  ps16Z: Pointer to store Z-axis value
  * @retval None
  */
void ICM20948_GetGyro(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    ICM20948_GyroRead(ps16X, ps16Y, ps16Z);
}

/**
  * @brief  Get accelerometer data
  * @param  ps16X: Pointer to store X-axis value
  * @param  ps16Y: Pointer to store Y-axis value
  * @param  ps16Z: Pointer to store Z-axis value
  * @retval None
  */
void ICM20948_GetAccel(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z)
{
    ICM20948_AccelRead(ps16X, ps16Y, ps16Z);
}
