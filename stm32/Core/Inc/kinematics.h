/**
  ******************************************************************************
  * @file    kinematics.h
  * @brief   Inverse kinematics solver for 4-DOF WeArm robotic arm
  *
  * Coordinate system:
  *   Origin: Intersection of base rotation axis with table surface (z=0 at table)
  *   X-axis: lateral (left/right)
  *   Y-axis: forward (primary reach direction)
  *   Z-axis: vertical upward along base rotation axis
 *   XY-plane: table surface
  *
  * Joints:
  *   Joint 0 (Servo 0): Base yaw rotation about Z-axis
  *   Joint 1 (Servo 1): Shoulder pitch from horizontal
  *   Joint 2 (Servo 2): Elbow pitch
  *   Joint 3 (Servo 3): Wrist pitch of end-effector
  ******************************************************************************
  */
#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Kinematics return codes */
#define KIN_OK                    0   /* Solution found */
#define KIN_ERR_Z_OUT_OF_RANGE    1   /* Z coordinate too low (below base) */
#define KIN_ERR_UNREACHABLE       2   /* Position too far (exceeds arm length) */
#define KIN_ERR_ELBOW_ACOS        3   /* Elbow angle calculation error */
#define KIN_ERR_ELBOW_LIMIT       4   /* Elbow angle out of range */
#define KIN_ERR_SHOULDER_ACOS     5   /* Shoulder angle calculation error */
#define KIN_ERR_SHOULDER_LIMIT    6   /* Shoulder angle out of range */
#define KIN_ERR_WRIST_LIMIT       7   /* Wrist angle out of range */

/* Kinematics data structure */
typedef struct {
    float L0;               /* Base height (shoulder offset), scaled x10 */
    float L1;               /* Upper arm length (shoulder to elbow), scaled x10 */
    float L2;               /* Forearm length (elbow to wrist), scaled x10 */
    float L3;               /* Gripper length (wrist to end-effector), scaled x10 */

    float servo_angle[6];   /* Calculated joint angles [deg]: [0]=base, [1]=shoulder, [2]=elbow, [3]=wrist */
    int16_t servo_pwm[6];   /* Calculated PWM values [us]: 500~2500, center at 1500 */
} kinematics_t;

/* Error code string lookup */
const char* kinematics_error_str(int code);

/* Setup link lengths (in mm) */
void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *kinematics);

/*
 * Core inverse kinematics solver.
 *
 * @param x          Target X coordinate [mm]
 * @param y          Target Y coordinate [mm]
 * @param z          Target Z coordinate [mm]
 * @param Alpha      Wrist angle relative to horizontal [deg], negative = downward
 *                   Recommended range: -25 to -65 deg
 * @param kinematics Pointer to kinematics instance (stores result in servo_angle[] and servo_pwm[])
 * @return           KIN_OK (0) on success, or error code on failure
 */
int kinematics_analysis(float x, float y, float z, float Alpha, kinematics_t *kinematics);

/*
 * Convenience function: search for valid Alpha and move all servos.
 * Sweeps Alpha from 0 to -135 degrees, picks the first valid solution.
 *
 * @param x          Target X coordinate [mm]
 * @param y          Target Y coordinate [mm]
 * @param z          Target Z coordinate [mm]
 * @param kinematics Pointer to kinematics instance
 * @return           0 = no valid position found, 1 = success
 */
int kinematics_move(float x, float y, float z, kinematics_t *kinematics);

#ifdef __cplusplus
}
#endif

#endif /* __KINEMATICS_H__ */
