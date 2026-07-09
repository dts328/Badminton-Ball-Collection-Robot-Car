/**
  ******************************************************************************
  * @file    kinematics.c
  * @brief   Inverse kinematics solver for 4-DOF WeArm robotic arm
  *
  * Ported from WeArm reference project (x_kinematics.c).
  * Adapted for STM32F4 HAL with FPU acceleration (Cortex-M4).
  *
  * ======================== Coordinate System ========================
  *
  *   Origin O: Intersection of base rotation axis with table surface
  *   X-axis:   Lateral direction (left/right)
  *   Y-axis:   Forward direction (primary reach)
  *   Z-axis:   Vertical upward along base rotation axis, 0 at table surface
  *   XY-plane: Table surface
  *
  *   The arm primarily reaches into the +Y half-plane.
  *
  * ==================== Mechanical Structure ====================
  *
  *        Z
  *        ^
  *        |   Joint 0 (Base): Rotation about Z-axis (yaw)
  *        |      Servo 0, mounted at origin
  *        |      Angle θ0: 0° = +Y direction, positive = toward +X
  *        |
  *        |   Joint 1 (Shoulder): Pitch from horizontal plane
  *        |      Servo 1, at height L0 above table surface
  *        |      Angle θ1: 0° = horizontal, positive = upward
  *        |
  *        |   L1 (105mm): Upper arm, shoulder → elbow
  *        |        \
  *        |         \  θ1
  *        |          O---  Joint 2 (Elbow): Pitch
  *        |               Servo 2, angle θ2
  *        |               θ2: 0° = straight (in line with L1), positive = bend up
  *        |
  *        |   L2 (98mm): Forearm, elbow → wrist
  *        |               \
  *        |                \  θ2
  *        |                 O---  Joint 3 (Wrist): End-effector pitch
  *   L0   |                      Servo 3, angle θ3 (Alpha = θ1+θ2+θ3)
  *        |                      θ3: 0° = horizontal, positive = upward
  *        |
  *        |   L3 (150mm): Gripper, wrist → end-effector tip
  *        |
  *   -----+------------------------> Y
  *        O
  *
  *   Total reach (arm fully extended horizontally):
  *     R_max = L1 + L2 + L3 = 105 + 98 + 150 = 353 mm
  *     at Z = L0 (shoulder height above table)
  *
  *   Working envelope (approximate):
  *     Y range: 0 ~ 353 mm (from base)
  *     Z range: -60 ~ 443 mm (L0 + L1 + L2 + L3)
  *
  * ==================== IK Algorithm ====================
  *
  *   Given target (x, y, z) and wrist angle Alpha (relative to horizontal):
  *
  *   1. Base rotation (Joint 0):
  *        θ0 = atan2(x, y)    [with sign handling for quadrant]
  *
  *   2. Project to 2D plane:
  *        r = sqrt(x² + y²)   [horizontal distance from base axis]
  *
  *   3. Subtract wrist offset (L3 at angle Alpha):
  *        r' = r - L3·cos(Alpha)
  *        z' = z - L0 - L3·sin(Alpha)
  *
  *   4. Solve 2-link planar arm (L1, L2) to reach (r', z'):
  *        Using Law of Cosines:
  *        cos(θ_elbow) = (r'² + z'² - L1² - L2²) / (2·L1·L2)
  *        θ2 = π - acos(cos(θ_elbow))
  *
  *        φ = atan2(z', r')   [angle of vector to wrist]
  *        ψ = acos((r'² + z'² + L1² - L2²) / (2·L1·√(r'² + z'²)))
  *        θ1 = φ + ψ          [for z' >= 0, elbow-up]
  *
  *   5. Wrist angle (Joint 3):
  *        θ3 = Alpha - θ1 + θ2   [(Alpha = θ1 - θ2 + θ3)]
  *
  *   6. Joint angle outputs:
  *        servo_angle[0] = θ0                  [base yaw]
  *        servo_angle[1] = -θ1               [shoulder: PWM 1500=0°(low)→500=135°(high)]
  *        servo_angle[2] = θ2                  [elbow]
  *        servo_angle[3] = θ3                  [wrist]
  *
  * ==================== Angle-to-PWM Conversion ====================
  *
  *   Servo pulse range: 500us ~ 2500us, center at 1500us
  *   Full angular range: ±135° = 270° total
  *
  *   For positive-direction servos (counterclockwise = increasing PWM):
  *     PWM = 1500 + 2000 * angle / 270
  *
  *   For negative-direction servos (counterclockwise = decreasing PWM):
  *     PWM = 1500 - 2000 * angle / 270
  *
  *   Direction assignments:
  *     Servo 0 (base):  negative  →  PWM = 1500 - 2000*θ0/270
  *     Servo 1 (shoulder): positive →  PWM = 1500 + 2000*θ1/270
  *     Servo 2 (elbow): positive  →  PWM = 1500 + 2000*θ2/270
  *     Servo 3 (wrist):  negative  →  PWM = 1500 - 2000*θ3/270
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "kinematics.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Private constants ---------------------------------------------------------*/
#define PI  3.14159265358979f

/* Error code string lookup table */
static const char* err_strings[] = {
    "OK",                       /* 0: KIN_OK */
    "Z out of range (too low)", /* 1: KIN_ERR_Z_OUT_OF_RANGE */
    "Position unreachable",     /* 2: KIN_ERR_UNREACHABLE */
    "Elbow acos error",         /* 3: KIN_ERR_ELBOW_ACOS */
    "Elbow angle limit",        /* 4: KIN_ERR_ELBOW_LIMIT */
    "Shoulder acos error",      /* 5: KIN_ERR_SHOULDER_ACOS */
    "Shoulder angle limit",     /* 6: KIN_ERR_SHOULDER_LIMIT */
    "Wrist angle limit",        /* 7: KIN_ERR_WRIST_LIMIT */
};

/* Public functions ----------------------------------------------------------*/

const char* kinematics_error_str(int code)
{
    if (code < 0 || code > 7) {
        return "Unknown error";
    }
    return err_strings[code];
}

void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *kinematics)
{
    /* Scale by 10x for internal calculation precision (0.1mm units) */
    kinematics->L0 = L0 * 10.0f;
    kinematics->L1 = L1 * 10.0f;
    kinematics->L2 = L2 * 10.0f;
    kinematics->L3 = L3 * 10.0f;

    /* Initialize angle and PWM arrays */
    memset(kinematics->servo_angle, 0, sizeof(kinematics->servo_angle));
    memset(kinematics->servo_pwm, 0, sizeof(kinematics->servo_pwm));
}

int kinematics_analysis(float x, float y, float z, float Alpha,
                        kinematics_t *kinematics)
{
    float theta3, theta4, theta5, theta6;
    float l0, l1, l2, l3;
    float aaa, bbb, ccc, zf_flag;

    /* Scale inputs by 10x */
    x = x * 10.0f;
    y = y * 10.0f;
    z = z * 10.0f;

    l0 = kinematics->L0;
    l1 = kinematics->L1;
    l2 = kinematics->L2;
    l3 = kinematics->L3;

    /* ===== Step 1: Base rotation (Joint 0) ===== */
    if (x == 0.0f) {
        theta6 = 0.0f;
    } else if (x > 0.0f && y < 0.0f) {
        /* Quadrant II: x>0, y<0 */
        theta6 = atanf(x / y);
        theta6 = 180.0f + (theta6 * 180.0f / PI);
    } else {
        if (y >= 0.0f) {
            /* Quadrant I or IV with y>=0 */
            theta6 = atanf(x / y);
            theta6 = theta6 * 180.0f / PI;
        } else {
            /* y < 0, x < 0 */
            theta6 = atanf(x / y);
            theta6 = theta6 * 180.0f / PI;
            theta6 = theta6 - 180.0f;
        }
    }

    /* ===== Step 2: Project to 2D plane ===== */
    /* r = horizontal distance from base axis */
    float r = sqrtf(x * x + y * y);

    /* ===== Step 3: Subtract wrist/gripper offset ===== */
    float alpha_rad = Alpha * PI / 180.0f;
    float r_prime = r - l3 * cosf(alpha_rad);   /* Effective horizontal reach */
    float z_prime = z - l0 - l3 * sinf(alpha_rad); /* Effective height */

    /* Check if Z is below base level */
    if (z_prime < -l0) {
        return KIN_ERR_Z_OUT_OF_RANGE;
    }

    /* Check if position is within reach */
    if (sqrtf(r_prime * r_prime + z_prime * z_prime) > (l1 + l2)) {
        return KIN_ERR_UNREACHABLE;
    }

    /* ===== Step 4: Solve 2-link planar arm ===== */

    /* Angle of wrist vector from horizontal */
    float r_prime_z_norm = sqrtf(r_prime * r_prime + z_prime * z_prime);
    if (r_prime_z_norm < 0.01f) {
        /* Degenerate case: wrist directly above shoulder */
        ccc = PI / 2.0f;
    } else {
        ccc = acosf(r_prime / r_prime_z_norm);
    }

    /* Shoulder angle contribution from law of cosines */
    bbb = (r_prime * r_prime + z_prime * z_prime + l1 * l1 - l2 * l2)
          / (2.0f * l1 * r_prime_z_norm);
    if (bbb > 1.0f || bbb < -1.0f) {
        return KIN_ERR_SHOULDER_ACOS;
    }

    /* Sign for z: positive z means arm reaches upward */
    if (z_prime < 0.0f) {
        zf_flag = -1.0f;
    } else {
        zf_flag = 1.0f;
    }

    theta5 = ccc * zf_flag + acosf(bbb);
    theta5 = theta5 * 180.0f / PI;   /* Convert to degrees */

    /* Validate shoulder angle */
    if (theta5 > 180.0f || theta5 < 0.0f) {
        return KIN_ERR_SHOULDER_LIMIT;
    }

    /* Elbow angle from law of cosines */
    aaa = -(r_prime * r_prime + z_prime * z_prime - l1 * l1 - l2 * l2)
          / (2.0f * l1 * l2);
    if (aaa > 1.0f || aaa < -1.0f) {
        return KIN_ERR_ELBOW_ACOS;
    }

    theta4 = acosf(aaa);
    theta4 = 180.0f - theta4 * 180.0f / PI;   /* Convert: elbow angle from straight */

    /* Validate elbow angle */
    if (theta4 > 135.0f || theta4 < -135.0f) {
        return KIN_ERR_ELBOW_LIMIT;
    }

    /* ===== Step 5: Wrist angle (Joint 3) ===== */
    theta3 = Alpha - theta5 + theta4;

    /* Validate wrist angle */
    if (theta3 > 90.0f || theta3 < -90.0f) {
        return KIN_ERR_WRIST_LIMIT;
    }

    /* ===== Step 6: Store joint angles ===== */
    kinematics->servo_angle[0] = theta6;        /* Base yaw */
    kinematics->servo_angle[1] = -theta5;  /* Shoulder: PWM 1500=0°(lowest), PWM 500=135°(highest); physical = -servo_angle */
    kinematics->servo_angle[2] = theta4;         /* Elbow */
    kinematics->servo_angle[3] = theta3;         /* Wrist */

    /* ===== Step 7: Convert angles to PWM values ===== */
    /* Servo 0 (base): negative direction */
    kinematics->servo_pwm[0] = (int16_t)(1500.0f - 2000.0f * kinematics->servo_angle[0] / 270.0f);

    /* Servo 1 (shoulder): positive direction */
    kinematics->servo_pwm[1] = (int16_t)(1500.0f + 2000.0f * kinematics->servo_angle[1] / 270.0f);

    /* Servo 2 (elbow): positive direction */
    kinematics->servo_pwm[2] = (int16_t)(1500.0f + 2000.0f * kinematics->servo_angle[2] / 270.0f);

    /* Servo 3 (wrist): negative direction */
    kinematics->servo_pwm[3] = (int16_t)(1500.0f - 2000.0f * kinematics->servo_angle[3] / 270.0f);

    return KIN_OK;
}

int kinematics_move(float x, float y, float z, kinematics_t *kinematics)
{
    int alpha;
    int result;
    int found = 0;

    /*
     * Sweep Alpha (wrist angle relative to horizontal) from +90 to -135 degrees.
     * Positive Alpha = wrist points up, Negative Alpha = wrist points down.
     * We search for the solution that gives the most natural elbow-up pose.
     *
     * Step size: 5 degrees
     */
    for (alpha = 90; alpha >= -135; alpha -= 5)
    {
        result = kinematics_analysis(x, y, z, (float)alpha, kinematics);
        if (result == KIN_OK)
        {
            found = 1;
            /* Keep searching — prefer more negative Alpha (elbow-up pose) */
        }
    }

    return found;
}
