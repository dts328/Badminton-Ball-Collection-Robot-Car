#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"

/* Protocol buffer size */
#define PROTOCOL_RX_BUF_SIZE    256
#define PROTOCOL_TX_BUF_SIZE    512

/* Command types */
typedef enum
{
    CMD_NONE = 0,
    CMD_MV,         /* $MV:vx,vy,vz! - Mecanum velocity */
    CMD_MSTOP,      /* $MSTOP! - Mecanum stop */
    CMD_MPID,       /* $MPID:kp,ki! - Set PID parameters */
    CMD_MENC,       /* $MENC! - Read encoders */
    CMD_MSTATUS,    /* $MSTATUS! - Status report */
    CMD_KMS,        /* $KMS:x,y,z,t! - Kinematics move (arm) */
    CMD_DST,        /* $DST! - Stop servos */
    CMD_DJR,        /* $DJR! - Reset servos */
    CMD_DRS,        /* $DRS! - Report joint states */
    CMD_GETA,       /* $GETA! - Ping alive */
    CMD_SERVO,      /* #IDPpwmTtime! - Servo control */
} CmdType_t;

/* External variables */
extern float Cmd_Vx, Cmd_Vy, Cmd_Vz;

/* Function prototypes */
void Protocol_Init(void);
void Protocol_Process(void);
void Protocol_SendString(const char *str);
void Protocol_SendStatus(void);
void Protocol_SendEncoder(void);

#endif /* __PROTOCOL_H */
