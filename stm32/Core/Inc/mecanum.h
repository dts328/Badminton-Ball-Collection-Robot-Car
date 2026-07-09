#ifndef __MECANUM_H
#define __MECANUM_H

#include "main.h"

/* Physical parameters */
#define WHEEL_DIAMETER      0.097f      /* Wheel diameter in meters */
#define WHEEL_PERIMETER     (WHEEL_DIAMETER * 3.14159265f)
#define WHEEL_SPACING       0.096f      /* Left/right wheel center spacing (half) in meters */
#define AXLE_SPACING        0.060f      /* Front/rear wheel center spacing (half) in meters */
#define ENCODER_PRECISION    122880.0f   /* 1024 * 4 * 30 */
#define CONTROL_FREQUENCY   100         /* 100Hz control frequency */

/* Encoder structure */
typedef struct
{
    int A;
    int B;
    int C;
    int D;
} Encoder_t;

/* Smooth control structure */
typedef struct
{
    float VX;
    float VY;
    float VZ;
} Smooth_Control_t;

/* External variables */
extern Encoder_t OriginalEncoder;
extern float Move_X, Move_Y, Move_Z;
extern Smooth_Control_t smooth_control;
extern int Motor_A, Motor_B, Motor_C, Motor_D;
extern float Target_A, Target_B, Target_C, Target_D;
extern float Encoder_A, Encoder_B, Encoder_C, Encoder_D;

/* Function prototypes */
void Mecanum_Init(void);
void Drive_Motor(float Vx, float Vy, float Vz);
void Get_Velocity_Form_Encoder(void);
void Smooth_Control(float vx, float vy, float vz);
float target_limit_float(float insert, float low, float high);
float float_abs(float insert);

#endif /* __MECANUM_H */
