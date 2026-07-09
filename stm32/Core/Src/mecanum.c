/**
  ******************************************************************************
  * @file    mecanum.c
  * @brief   Mecanum wheel kinematics module
  ******************************************************************************
  */

#include "mecanum.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define VELOCITY_MAX        3.5f    /* Maximum velocity (m/s) */
#define SMOOTH_STEP         1.0f    /* Smooth control step (更快响应) */
#define SMOOTH_DECAY        0.8f    /* Smooth control decay factor (更快衰减) */

/* Encoder raw data */
Encoder_t OriginalEncoder;

/* Movement variables */
float Move_X = 0.0f, Move_Y = 0.0f, Move_Z = 0.0f;

/* Smooth control */
Smooth_Control_t smooth_control;

/* Motor PWM values */
int Motor_A = 0, Motor_B = 0, Motor_C = 0, Motor_D = 0;

/* Target speeds for each motor */
float Target_A = 0.0f, Target_B = 0.0f, Target_C = 0.0f, Target_D = 0.0f;

/* Encoder speeds for each motor */
float Encoder_A = 0.0f, Encoder_B = 0.0f, Encoder_C = 0.0f, Encoder_D = 0.0f;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Clamp float value to range
  */
static float clampf(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
  * @brief  Check if float is valid
  */
static uint8_t is_valid_float(float value)
{
    return !isnan(value) && !isinf(value);
}

/**
  * @brief  Smooth control step function
  */
static float smooth_step(float current, float target, float step)
{
    if (target > current) {
        current += step;
        if (current > target) current = target;
    } else if (target < current) {
        current -= step;
        if (current < target) current = target;
    }
    return current;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize mecanum wheel module
  */
void Mecanum_Init(void)
{
    Move_X = 0.0f;
    Move_Y = 0.0f;
    Move_Z = 0.0f;
    
    Motor_A = 0;
    Motor_B = 0;
    Motor_C = 0;
    Motor_D = 0;
    
    Target_A = 0.0f;
    Target_B = 0.0f;
    Target_C = 0.0f;
    Target_D = 0.0f;
    
    Encoder_A = 0.0f;
    Encoder_B = 0.0f;
    Encoder_C = 0.0f;
    Encoder_D = 0.0f;
    
    smooth_control.VX = 0.0f;
    smooth_control.VY = 0.0f;
    smooth_control.VZ = 0.0f;
}

/**
  * @brief  Inverse kinematics - calculate target speed for each wheel
  * @param  Vx: Forward/backward speed (m/s), positive = forward
  * @param  Vy: Left/right speed (m/s), positive = left
  * @param  Vz: Rotation speed (rad/s), positive = counterclockwise
  */
void Drive_Motor(float Vx, float Vy, float Vz)
{
    /* Validate inputs */
    if (!is_valid_float(Vx)) Vx = 0.0f;
    if (!is_valid_float(Vy)) Vy = 0.0f;
    if (!is_valid_float(Vz)) Vz = 0.0f;
    
    /* Limit input speeds */
    Vx = clampf(Vx, -VELOCITY_MAX, VELOCITY_MAX);
    Vy = clampf(Vy, -VELOCITY_MAX, VELOCITY_MAX);
    Vz = clampf(Vz, -VELOCITY_MAX, VELOCITY_MAX);
    
    /* Smooth control */
    smooth_control.VX = smooth_step(smooth_control.VX, Vx, SMOOTH_STEP);
    smooth_control.VY = smooth_step(smooth_control.VY, Vy, SMOOTH_STEP);
    smooth_control.VZ = smooth_step(smooth_control.VZ, Vz, SMOOTH_STEP);
    
    /* Decay when target is zero */
    if (Vx == 0.0f) smooth_control.VX *= SMOOTH_DECAY;
    if (Vy == 0.0f) smooth_control.VY *= SMOOTH_DECAY;
    if (Vz == 0.0f) smooth_control.VZ *= SMOOTH_DECAY;
    
    /* Get smoothed values */
    Vx = smooth_control.VX;
    Vy = smooth_control.VY;
    Vz = smooth_control.VZ;
    
    /* Inverse kinematics for mecanum wheel */
    /* Layout (top view):
     *   C ---- A  (Front)
     *   |      |
     *   B ---- D  (Rear)
     *   
     *   A = Right Front
     *   B = Left Rear
     *   C = Left Front
     *   D = Right Rear
     */
    Target_A = Vx - Vy - Vz * (AXLE_SPACING + WHEEL_SPACING);
    Target_B = Vx + Vy - Vz * (AXLE_SPACING + WHEEL_SPACING);
    Target_C = Vx + Vy + Vz * (AXLE_SPACING + WHEEL_SPACING);
    Target_D = Vx - Vy + Vz * (AXLE_SPACING + WHEEL_SPACING);
    
    /* Limit wheel target speeds */
    Target_A = clampf(Target_A, -VELOCITY_MAX, VELOCITY_MAX);
    Target_B = clampf(Target_B, -VELOCITY_MAX, VELOCITY_MAX);
    Target_C = clampf(Target_C, -VELOCITY_MAX, VELOCITY_MAX);
    Target_D = clampf(Target_D, -VELOCITY_MAX, VELOCITY_MAX);
}

/**
  * @brief  Get velocity from encoder readings
  * @note   Converts encoder counts to speed in m/s
  */
void Get_Velocity_Form_Encoder(void)
{
    int16_t enc_a, enc_b, enc_c, enc_d;
    float speed_factor;
    
    /* Read encoder raw values (atomic read+clear) */
    enc_a = Read_Encoder(2);
    enc_b = Read_Encoder(3);
    enc_c = Read_Encoder(4);
    enc_d = Read_Encoder(5);
    
    /* Store original values */
    OriginalEncoder.A = enc_a;
    OriginalEncoder.B = enc_b;
    OriginalEncoder.C = enc_c;
    OriginalEncoder.D = enc_d;
    
    /* Calculate speed factor: counts -> m/s */
    /* speed = counts * frequency * perimeter / precision */
    speed_factor = CONTROL_FREQUENCY * WHEEL_PERIMETER / ENCODER_PRECISION;
    
    /* Convert and apply polarity for mecanum car */
    Encoder_A = (float)enc_a * speed_factor;   /* Normal */
    Encoder_B = (float)(-enc_b) * speed_factor;  /* Inverted */
    Encoder_C = (float)(-enc_c) * speed_factor;  /* Inverted */
    Encoder_D = (float)enc_d * speed_factor;    /* Normal */
    
    /* Validate outputs */
    if (!is_valid_float(Encoder_A)) Encoder_A = 0.0f;
    if (!is_valid_float(Encoder_B)) Encoder_B = 0.0f;
    if (!is_valid_float(Encoder_C)) Encoder_C = 0.0f;
    if (!is_valid_float(Encoder_D)) Encoder_D = 0.0f;
}

/**
  * @brief  Limit float value
  */
float target_limit_float(float insert, float low, float high)
{
    return clampf(insert, low, high);
}

/**
  * @brief  Get absolute value of float
  */
float float_abs(float insert)
{
    return insert >= 0.0f ? insert : -insert;
}
