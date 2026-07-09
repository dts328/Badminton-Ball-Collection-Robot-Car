/**
  ******************************************************************************
  * @file    servo.h
  * @brief   Servo control interface using STM32F4 hardware PWM
  *
  * PWM configuration (50 Hz, 20ms period):
  *   - TIM8 CH1-4 on PC6-PC9  (Servos 0-3: arm joints)
  *   - TIM12 CH1-2 on PB14-PB15 (Servos 4-5: gripper, spare)
  *
  * Pulse range: 500us (0.5ms) to 2500us (2.5ms), center at 1500us (1.5ms)
  * Period ARR = 20000-1, timer ticks = 1us at current prescaler
  ******************************************************************************
  */
#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

/* Number of servo channels */
#define SERVO_NUM         6

/* PWM limits (microseconds) */
#define SERVO_PWM_MIN     500
#define SERVO_PWM_MAX     2500
#define SERVO_PWM_CENTER  1500

/* PWM period (timer ticks) */
#define SERVO_PERIOD      20000

/* Servo-to-joint mapping */
#define SERVO_BASE        0    /* Joint 0: Base yaw rotation */
#define SERVO_SHOULDER    1    /* Joint 1: Shoulder pitch */
#define SERVO_ELBOW       2    /* Joint 2: Elbow pitch */
#define SERVO_WRIST       3    /* Joint 3: Wrist pitch */
#define SERVO_GRIPPER     4    /* Gripper */
#define SERVO_SPARE       5    /* Spare */

/* Joint names for reporting */
extern const char* joint_names[6];

/* Servo state structure for incremental movement */
typedef struct {
    int16_t cur;          /* Current PWM value */
    int16_t aim;          /* Target PWM value */
    int16_t inc;          /* Increment per update cycle (signed, 0.01 resolution) */
    uint16_t time;        /* Total movement time in ms */
    uint16_t elapsed;     /* Elapsed time in ms */
    uint16_t steps;       /* Total number of steps */
    uint16_t step_count;  /* Current step count */
} servo_state_t;

/* Global servo state array */
extern servo_state_t servo_state[SERVO_NUM];

/*
 * Initialize all servo PWM channels and start output.
 * Must be called after MX_TIM8_Init() and MX_TIM12_Init().
 * @param initial_pwm  Optional initial PWM for arm servos (4 values), NULL = use center (1500)
 */
void servo_init(const int16_t *initial_pwm);

/*
 * Immediately set a servo to a specific PWM value.
 * @param servo_id  0-5
 * @param pwm       Pulse width in us (500-2500), clamped if out of range
 */
void servo_set_pwm(uint8_t servo_id, int16_t pwm);

/*
 * Convert joint angle to PWM value.
 * @param servo_id  0-5
 * @param angle_deg Joint angle in degrees
 * @return          PWM value in us (500-2500)
 */
int16_t servo_angle_to_pwm(uint8_t servo_id, float angle_deg);

/*
 * Set up incremental move: servo moves from current to target over time_ms.
 * @param servo_id  0-5
 * @param target_pwm Target PWM value in us (500-2500)
 * @param time_ms    Movement duration in ms (0 = immediate)
 */
void servo_move_to(uint8_t servo_id, int16_t target_pwm, uint16_t time_ms);

/*
 * Move all 4 arm joints (servos 0-3) using incremental movement.
 * @param pwm    Array of 4 PWM values [base, shoulder, elbow, wrist]
 * @param time_ms Movement duration in ms
 */
void servo_move_arm(int16_t pwm[4], uint16_t time_ms);

/*
 * Stop all servos immediately (or a specific servo if id < 6).
 * Pass id=255 to stop all.
 */
void servo_stop(uint8_t servo_id);

/*
 * Reset all servos to center position (1500us) over time_ms.
 */
void servo_reset_all(uint16_t time_ms);

/*
 * Must be called periodically (e.g., every 20ms from SysTick).
 * Steps all incremental servo movements.
 */
void servo_update(void);

/*
 * Format joint states into a human-readable string.
 * Returns number of characters written to buf (excluding null terminator).
 */
int servo_get_state_string(char *buf, uint16_t buf_size);

/*
 * Format coordinate system description into a string.
 */
int servo_get_coord_sys_string(char *buf, uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H__ */
