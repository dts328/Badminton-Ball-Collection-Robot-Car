#include "i2c.h"

/* Private function prototypes -----------------------------------------------*/
static void delay_us(uint32_t us);

/**
  * @brief  I2C GPIO initialization
  * @param  None
  * @retval None
  */
void I2C_GPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* Enable GPIOB clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure PB10 (SCL) and PB11 (SDA) as output open-drain */
    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Set pins high */
    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
}

/**
  * @brief  I2C start signal
  * @param  None
  * @retval None
  */
void I2C_Start(void)
{
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    delay_us(1);
    I2C_SDA_LOW();
    delay_us(1);
    I2C_SCL_LOW();
    delay_us(1);
}

/**
  * @brief  I2C stop signal
  * @param  None
  * @retval None
  */
void I2C_Stop(void)
{
    I2C_SCL_LOW();
    I2C_SDA_LOW();
    delay_us(1);
    I2C_SCL_HIGH();
    delay_us(1);
    I2C_SDA_HIGH();
    delay_us(1);
}

/**
  * @brief  Wait for I2C acknowledge
  * @param  None
  * @retval true: ACK received, false: NACK
  */
bool I2C_WaitForAck(void)
{
    uint8_t timeout = 0;
    
    I2C_SDA_HIGH();
    delay_us(1);
    I2C_SCL_HIGH();
    delay_us(1);
    
    while(I2C_SDA_READ())
    {
        timeout++;
        if(timeout > 50)
        {
            I2C_Stop();
            return false;
        }
        delay_us(1);
    }
    
    I2C_SCL_LOW();
    return true;
}

/**
  * @brief  Send I2C ACK
  * @param  None
  * @retval None
  */
void I2C_Ack(void)
{
    I2C_SCL_LOW();
    I2C_SDA_LOW();
    delay_us(1);
    I2C_SCL_HIGH();
    delay_us(1);
    I2C_SCL_LOW();
}

/**
  * @brief  Send I2C NACK
  * @param  None
  * @retval None
  */
void I2C_NAck(void)
{
    I2C_SCL_LOW();
    I2C_SDA_HIGH();
    delay_us(1);
    I2C_SCL_HIGH();
    delay_us(1);
    I2C_SCL_LOW();
}

/**
  * @brief  Write one byte via I2C
  * @param  Data: Byte to write
  * @retval None
  */
void I2C_WriteByte(uint8_t Data)
{
    uint8_t i;
    
    I2C_SCL_LOW();
    for(i = 0; i < 8; i++)
    {
        if(Data & 0x80)
            I2C_SDA_HIGH();
        else
            I2C_SDA_LOW();
        
        Data <<= 1;
        delay_us(1);
        I2C_SCL_HIGH();
        delay_us(1);
        I2C_SCL_LOW();
        delay_us(1);
    }
}

/**
  * @brief  Read one byte via I2C
  * @param  Ack: I2C_ACK to send ACK, I2C_NACK to send NACK
  * @retval Received byte
  */
uint8_t I2C_ReadByte(uint8_t Ack)
{
    uint8_t i, Data = 0;
    
    for(i = 0; i < 8; i++)
    {
        I2C_SCL_LOW();
        delay_us(1);
        I2C_SCL_HIGH();
        Data <<= 1;
        if(I2C_SDA_READ())
            Data |= 0x01;
        delay_us(1);
    }
    
    if(Ack == I2C_ACK)
        I2C_Ack();
    else
        I2C_NAck();
    
    return Data;
}

/**
  * @brief  Write one byte to register
  * @param  DevAddr: Device I2C address
  * @param  RegAddr: Register address
  * @param  Data: Data to write
  * @retval 1: success
  */
uint8_t I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
    I2C_WaitForAck();
    I2C_WriteByte(RegAddr);
    I2C_WaitForAck();
    I2C_WriteByte(Data);
    I2C_WaitForAck();
    I2C_Stop();
    return 1;
}

/**
  * @brief  Read one byte from register
  * @param  DevAddr: Device I2C address
  * @param  RegAddr: Register address
  * @retval Read data
  */
uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
    uint8_t Data = 0;
    
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
    I2C_WaitForAck();
    I2C_WriteByte(RegAddr);
    I2C_WaitForAck();
    
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
    I2C_WaitForAck();
    Data = I2C_ReadByte(I2C_NACK);
    I2C_Stop();
    
    return Data;
}

/**
  * @brief  Write specific bits to register
  * @param  DevAddr: Device I2C address
  * @param  RegAddr: Register address
  * @param  BitStart: Start bit position
  * @param  Length: Number of bits
  * @param  Data: Data to write
  * @retval true: success
  */
bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{
    uint8_t Dat, Mask;
    
    Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}

/**
  * @brief  Write one bit to register
  * @param  DevAddr: Device I2C address
  * @param  RegAddr: Register address
  * @param  BitNum: Bit position
  * @param  Data: 0 or 1
  * @retval true: success
  */
bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;
    
    Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}

/**
  * @brief  Write buffer to device
  * @param  DevAddr: Device I2C address
  * @param  RegAddr: Register address
  * @param  Num: Number of bytes
  * @param  pBuff: Data buffer
  * @retval true: success
  */
bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
    uint8_t i;
    
    if(Num == 0 || pBuff == NULL)
        return false;
    
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
    I2C_WaitForAck();
    I2C_WriteByte(RegAddr);
    I2C_WaitForAck();
    
    for(i = 0; i < Num; i++)
    {
        I2C_WriteByte(*(pBuff + i));
        I2C_WaitForAck();
    }
    I2C_Stop();
    
    return true;
}

/**
  * @brief  Read buffer from device
  * @param  DevAddr: Device I2C address
  * @param  RegAddr: Register address
  * @param  Num: Number of bytes
  * @param  pBuff: Buffer to store data
  * @retval true: success
  */
bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
    uint8_t i;
    
    if(Num == 0 || pBuff == NULL)
        return false;
    
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
    I2C_WaitForAck();
    I2C_WriteByte(RegAddr);
    I2C_WaitForAck();
    
    I2C_Start();
    I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
    I2C_WaitForAck();
    
    for(i = 0; i < Num; i++)
    {
        if((Num - 1) == i)
            *(pBuff + i) = I2C_ReadByte(I2C_NACK);
        else
            *(pBuff + i) = I2C_ReadByte(I2C_ACK);
    }
    
    I2C_Stop();
    
    return true;
}

/**
  * @brief  Delay in microseconds
  * @param  us: Microseconds to delay
  * @retval None
  * @note   Approximate delay for 168MHz clock
  */
static void delay_us(uint32_t us)
{
    uint32_t i;
    for(i = 0; i < us * 42; i++);  /* Approximate for 168MHz */
}
