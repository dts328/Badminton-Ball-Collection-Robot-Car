/**
  ******************************************************************************
  * @file    servo.c
  * @brief   Servo control using STM32F4 hardware PWM (HAL)
  *
  * Servo assignment:
  *   Servo 0 (Base)      - TIM8_CH1, PC6
  *   Servo 1 (Shoulder)  - TIM8_CH2, PC7
  *   Servo 2 (Elbow)     - TIM8_CH3, PC8
  *   Servo 3 (Wrist)     - TIM8_CH4, PC9
  *   Servo 4 (Gripper)   - TIM12_CH1, PB14
  *   Servo 5 (Spare)     - TIM12_CH2, PB15
  *
  * PWM: 50 Hz, period = 20000 us, pulse = 500~2500 us
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "servo.h"
#include <stdio.h>
#include <string.h>

/* Joint name strings */
const char* joint_names[6] = {
    "Base     ",   /* Servo 0 */
    "Shoulder ",   /* Servo 1 */
    "Elbow    ",   /* Servo 2 */
    "Wrist    ",   /* Servo 3 */
    "Gripper  ",   /* Servo 4 */
    "Spare    ",   /* Servo 5 */
};

/* Servo direction signs for angle-to-PWM conversion
 * +1 = angle increases → PWM increases
 * -1 = angle increases → PWM decreases
 */
static const int8_t servo_direction[6] = {
    -1,   /* Servo 0 (Base): negative direction */
    +1,   /* Servo 1 (Shoulder): positive direction */
    +1,   /* Servo 2 (Elbow): positive direction */
    -1,   /* Servo 3 (Wrist): negative direction */
    +1,   /* Servo 4 (Gripper): positive direction */
    +1,   /* Servo 5 (Spare): positive direction */
};

/* Global servo state array */
servo_state_t servo_state[SERVO_NUM];

/* Forward declaration of internal helper */
static void servo_hw_set(uint8_t servo_id, int16_t pwm);

/* ======================================================================== */
/*  Initialization                                                          */
/* ======================================================================== */

void servo_init(const int16_t *initial_pwm)
{
    uint8_t i;

    /* Initialize state array */
    for (i = 0; i < SERVO_NUM; i++) {
        servo_state[i].cur     = SERVO_PWM_CENTER;
        servo_state[i].aim     = SERVO_PWM_CENTER;
        servo_state[i].inc     = 0;
        servo_state[i].time    = 0;
        servo_state[i].elapsed = 0;
    }

    /* Start all PWM channels */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);   /* Servo 0: Base */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);   /* Servo 1: Shoulder */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);   /* Servo 2: Elbow */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);   /* Servo 3: Wrist */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  /* Servo 4: Gripper */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);  /* Servo 5: Spare */

    /* Set arm servos (0-3) to initial position or center */
    for (i = 0; i < 4; i++) {
        int16_t pwm = (initial_pwm != NULL) ? initial_pwm[i] : SERVO_PWM_CENTER;
        servo_state[i].cur = pwm;
        servo_state[i].aim = pwm;
        servo_hw_set(i, pwm);
    }
    servo_state[4].cur = 500;
    servo_state[4].aim = 500;
    servo_hw_set(4, 500);
    servo_state[5].cur = 500;
    servo_state[5].aim = 500;
    servo_hw_set(5, 500);
}

/* ======================================================================== */
/*  Hardware PWM Output                                                     */
/* ======================================================================== */

/*
 * Write PWM compare value to the hardware timer channel.
 * Clamps value to valid servo range.
 */
static void servo_hw_set(uint8_t servo_id, int16_t pwm)
{
    /* Clamp to valid range */
    if (pwm < SERVO_PWM_MIN)  pwm = SERVO_PWM_MIN;
    if (pwm > SERVO_PWM_MAX)  pwm = SERVO_PWM_MAX;

    switch (servo_id) {
        case 0:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint32_t)pwm);
            break;
        case 1:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint32_t)pwm);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint32_t)pwm);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, (uint32_t)pwm);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, (uint32_t)pwm);
            break;
        case 5:
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, (uint32_t)pwm);
            break;
        default:
            break;
    }
}

/* ======================================================================== */
/*  Immediate Servo Control                                                 */
/* ======================================================================== */

void servo_set_pwm(uint8_t servo_id, int16_t pwm)
{
    if (servo_id >= SERVO_NUM) return;

    servo_state[servo_id].cur     = pwm;
    servo_state[servo_id].aim     = pwm;
    servo_state[servo_id].inc     = 0;
    servo_state[servo_id].time    = 0;
    servo_state[servo_id].elapsed = 0;

    servo_hw_set(servo_id, pwm);
}

int16_t servo_angle_to_pwm(uint8_t servo_id, float angle_deg)
{
    int32_t pwm;

    /* PWM = 1500 +/- 2000 * angle / 270, direction determines sign */
    pwm = (int32_t)(1500.0f + servo_direction[servo_id] * 2000.0f * angle_deg / 270.0f);

    /* Clamp */
    if (pwm < SERVO_PWM_MIN)  pwm = SERVO_PWM_MIN;
    if (pwm > SERVO_PWM_MAX)  pwm = SERVO_PWM_MAX;

    return (int16_t)pwm;
}

/* ======================================================================== */
/*  Incremental Movement                                                    */
/* ======================================================================== */

void servo_move_to(uint8_t servo_id, int16_t target_pwm, uint16_t time_ms)
{
    if (servo_id >= SERVO_NUM) return;

    /* Clamp target */
    if (target_pwm < SERVO_PWM_MIN)  target_pwm = SERVO_PWM_MIN;
    if (target_pwm > SERVO_PWM_MAX)  target_pwm = SERVO_PWM_MAX;

    servo_state[servo_id].aim     = target_pwm;
    servo_state[servo_id].time    = time_ms;
    servo_state[servo_id].elapsed = 0;

    if (time_ms == 0) {
        /* Immediate move */
        servo_state[servo_id].cur = target_pwm;
        servo_state[servo_id].inc = 0;
        servo_hw_set(servo_id, target_pwm);
    } else {
        /*
         * Calculate increment per update cycle.
         * servo_update() is called every UPDATE_INTERVAL_MS (e.g., 20ms).
         * inc = (target - current) / (time_ms / UPDATE_INTERVAL_MS)
         * We store inc in units of 0.1 PWM-step per update for precision.
         * Actually: use a simpler approach — compute total needed steps.
         */
        int32_t diff = (int32_t)target_pwm - (int32_t)servo_state[servo_id].cur;
        int32_t steps = (int32_t)time_ms / 20;   /* 20ms per servo update cycle */

        if (steps < 1) steps = 1;
        if (steps > 10000) steps = 10000;        /* Safety cap */

        /* Store increment with 0.1 resolution:
         * servo_state.inc = (diff * 10) / steps  →  signed, in 0.1-unit steps
         */
        servo_state[servo_id].inc = (diff * 10) / steps;
    }
}

void servo_move_arm(int16_t pwm[4], uint16_t time_ms)
{
    uint8_t i;
    for (i = 0; i < 4; i++) {
        servo_move_to(i, pwm[i], time_ms);
    }
}

/* ======================================================================== */
/*  Stop / Reset                                                            */
/* ======================================================================== */

void servo_stop(uint8_t servo_id)
{
    uint8_t i;

    if (servo_id < SERVO_NUM) {
        /* Stop specific servo */
        servo_state[servo_id].aim = servo_state[servo_id].cur;
        servo_state[servo_id].inc = 0;
        servo_state[servo_id].time = 0;
        servo_state[servo_id].elapsed = 0;
    } else {
        /* servo_id == 255 means stop all */
        for (i = 0; i < SERVO_NUM; i++) {
            servo_state[i].aim = servo_state[i].cur;
            servo_state[i].inc = 0;
            servo_state[i].time = 0;
            servo_state[i].elapsed = 0;
        }
    }
}

void servo_reset_all(uint16_t time_ms)
{
    uint8_t i;
    for (i = 0; i < SERVO_NUM; i++) {
        servo_move_to(i, SERVO_PWM_CENTER, time_ms);
    }
}

/* ======================================================================== */
/*  Periodic Update (call every ~20ms)                                      */
/* ======================================================================== */

void servo_update(void)
{
    uint8_t i;

    for (i = 0; i < SERVO_NUM; i++) {
        if (servo_state[i].inc != 0 && servo_state[i].elapsed < servo_state[i].time) {

            /* Accumulate with 0.1 resolution */
            int32_t new_cur = (int32_t)servo_state[i].cur * 10 + servo_state[i].inc;

            /* Check if reached or passed target */
            int32_t target_x10 = (int32_t)servo_state[i].aim * 10;

            if ((servo_state[i].inc > 0 && new_cur >= target_x10) ||
                (servo_state[i].inc < 0 && new_cur <= target_x10)) {
                /* Reached target */
                servo_state[i].cur = servo_state[i].aim;
                servo_state[i].inc = 0;
                servo_state[i].elapsed = servo_state[i].time;
            } else {
                servo_state[i].cur = (int16_t)(new_cur / 10);
                servo_state[i].elapsed += 20;
            }

            servo_hw_set(i, servo_state[i].cur);
        }
    }
}

/* ======================================================================== */
/*  State Reporting                                                         */
/* ======================================================================== */

int servo_get_state_string(char *buf, uint16_t buf_size)
{
    int len = 0;

    len += snprintf(buf + len, buf_size - len,
        "==================================================\r\n");
    len += snprintf(buf + len, buf_size - len,
        "          WeArm Joint State Report\r\n");
    len += snprintf(buf + len, buf_size - len,
        "==================================================\r\n");
    len += snprintf(buf + len, buf_size - len,
        "Joint | Servo |   Angle[deg] |  PWM[us] | State\r\n");
    len += snprintf(buf + len, buf_size - len,
        "------|-------|--------------|----------|-------\r\n");
    len += snprintf(buf + len, buf_size - len,
        "Base     |   0  |  (direct PWM)|  %5d  | %s\r\n",
        servo_state[0].cur,
        servo_state[0].inc == 0 ? "STOP" : "MOVE");
    len += snprintf(buf + len, buf_size - len,
        "Shoulder |   1  |  (direct PWM)|  %5d  | %s\r\n",
        servo_state[1].cur,
        servo_state[1].inc == 0 ? "STOP" : "MOVE");
    len += snprintf(buf + len, buf_size - len,
        "Elbow    |   2  |  (direct PWM)|  %5d  | %s\r\n",
        servo_state[2].cur,
        servo_state[2].inc == 0 ? "STOP" : "MOVE");
    len += snprintf(buf + len, buf_size - len,
        "Wrist    |   3  |  (direct PWM)|  %5d  | %s\r\n",
        servo_state[3].cur,
        servo_state[3].inc == 0 ? "STOP" : "MOVE");
    len += snprintf(buf + len, buf_size - len,
        "Gripper  |   4  |  (direct PWM)|  %5d  | %s\r\n",
        servo_state[4].cur,
        servo_state[4].inc == 0 ? "STOP" : "MOVE");
    len += snprintf(buf + len, buf_size - len,
        "Spare    |   5  |  (direct PWM)|  %5d  | %s\r\n",
        servo_state[5].cur,
        servo_state[5].inc == 0 ? "STOP" : "MOVE");
    len += snprintf(buf + len, buf_size - len,
        "==================================================\r\n");

    return len;
}

int servo_get_coord_sys_string(char *buf, uint16_t buf_size)
{
    int len = 0;

    len += snprintf(buf + len, buf_size - len,
        "==================================================\r\n");
    len += snprintf(buf + len, buf_size - len,
        "     WeArm Coordinate System & Kinematics\r\n");
    len += snprintf(buf + len, buf_size - len,
        "==================================================\r\n");
    len += snprintf(buf + len, buf_size - len,
        "\r\n");
    len += snprintf(buf + len, buf_size - len,
        "Coordinate System:\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Origin O: Intersection of base rotation axis with table\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  X-axis:   Lateral (left/right)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Y-axis:   Forward (primary reach direction)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Z-axis:   Vertical upward along base rotation axis\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  XY-plane: Table surface (z=0 at table)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "\r\n");
    len += snprintf(buf + len, buf_size - len,
        "Joint Definitions (DH-like parameters):\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Joint 0 (Base):     Revolute, axis = Z\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Rotation about Z-axis, controls azimuth\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Angle range: +/-135 deg\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Joint 1 (Shoulder): Revolute, axis = X\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Pitch from horizontal plane\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Offset: L0 vertical from table surface\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Angle range: 0 to 180 deg\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Joint 2 (Elbow):    Revolute, axis = X\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Pitch relative to upper arm\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Link L1 = 105mm (shoulder to elbow)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Angle range: +/-135 deg\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Joint 3 (Wrist):    Revolute, axis = X\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Pitch of end-effector relative to horizontal\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Link L2 = 98mm (elbow to wrist)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "    Angle range: +/-90 deg\r\n");
    len += snprintf(buf + len, buf_size - len,
        "\r\n");
    len += snprintf(buf + len, buf_size - len,
        "End Effector:\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Link L3 = 150mm (wrist to gripper tip)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "\r\n");
    len += snprintf(buf + len, buf_size - len,
        "Workspace (approximate):\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Max horizontal reach: 353mm (L1+L2+L3)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Max vertical reach:   443mm (L0+L1+L2+L3)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Base height (L0):      see setup_kinematics()\r\n");
    len += snprintf(buf + len, buf_size - len,
        "\r\n");
    len += snprintf(buf + len, buf_size - len,
        "Inverse Kinematics Method:\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  Geometric approach using Law of Cosines\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  1. Base rotation via atan2(x, y)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  2. Project to 2D plane (r, z)\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  3. Subtract wrist offset via Alpha parameter\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  4. Solve 2-link arm with Law of Cosines\r\n");
    len += snprintf(buf + len, buf_size - len,
        "  5. Wrist angle = Alpha - Shoulder + Elbow\r\n");
    len += snprintf(buf + len, buf_size - len,
        "==================================================\r\n");

    return len;
}
