/**
  ******************************************************************************
  * @file    ros_protocol.h
  * @brief   ROS binary protocol for mecanum car and robotic arm control
  ******************************************************************************
  */

#ifndef __ROS_PROTOCOL_H
#define __ROS_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Exported defines ----------------------------------------------------------*/
#define ROS_FRAME_HEADER        0x7B    /* Frame header */
#define ROS_FRAME_TAIL          0x7D    /* Frame tail */
#define ROS_RX_FRAME_LEN        11      /* Receive frame length */
#define ROS_TX_FRAME_LEN        26      /* Transmit frame length (v2: 4 wheels + IMU + battery) */

/* Control modes */
#define ROS_MODE_MECANUM        0x00    /* Mecanum wheel control */
#define ROS_MODE_ARM            0x01    /* Robotic arm control */
#define ROS_MODE_AUTO_CHARGE    0x02    /* Auto charging mode */
#define ROS_MODE_IP_FRAME       0xFF    /* IP configuration frame */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  ROS receive data structure
  */
typedef struct
{
    uint8_t Frame_Header;       /* Frame header (0x7B) */
    uint8_t Mode;               /* Control mode */
    uint8_t Reserved;           /* Reserved byte */
    int16_t X_speed;            /* X speed or J1 angle (mm/s or degree) */
    int16_t Y_speed;            /* Y speed or J2 angle (mm/s or degree) */
    int16_t Z_speed;            /* Z speed or J3 angle (mm/s or degree) */
    uint8_t Check_Sum;          /* XOR checksum */
    uint8_t Frame_Tail;         /* Frame tail (0x7D) */
} ROS_Receive_t;

/**
  * @brief  ROS receive buffer union
  */
typedef union
{
    uint8_t buffer[ROS_RX_FRAME_LEN];
    ROS_Receive_t data;
} ROS_RxBuffer_t;

/* Note: TX v2 uses manual serialization (ROS_WriteBE16), 26 bytes frame */
/* TX frame: [0x7B][flags][A_H][A_L][B_H][B_L][C_H][C_L][D_H][D_L]
              [accX_H][accX_L][accY_H][accY_L][accZ_H][accZ_L]
              [gyroX_H][gyroX_L][gyroY_H][gyroY_L][gyroZ_H][gyroZ_L]
              [bat_H][bat_L][checksum][0x7D] */

/* Exported variables --------------------------------------------------------*/
extern volatile uint8_t ros_rx_flag;        /* ROS receive complete flag */
extern ROS_RxBuffer_t ros_rx_buf;           /* ROS receive buffer */
extern float ROS_Move_X, ROS_Move_Y, ROS_Move_Z;  /* ROS target velocity */
extern uint8_t ROS_Control_Mode;            /* Current control mode */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize ROS protocol
  */
void ROS_Init(void);

/**
  * @brief  Process received byte from USART3
  * @param  data: Received byte
  */
void ROS_ProcessReceive(uint8_t data);

/**
  * @brief  Send status frame to ROS (20Hz)
  */
void ROS_SendStatus(void);

/**
  * @brief  Process ROS receive data
  * @note   Call this in main loop after ros_rx_flag is set
  */
void ROS_ProcessData(void);

/**
  * @brief  Calculate XOR checksum
  * @param  buf: Data buffer
  * @param  len: Data length (excluding checksum byte)
  * @retval Checksum value
  */
uint8_t ROS_CalcChecksum(uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __ROS_PROTOCOL_H */
