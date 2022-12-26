/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task
  * @note       1.remember to add detect_task.c/h
  *             2.remember to add INS_task.c/h
  *             3.remember to add chassis_power_control.c/h
  *             4.remember to add gimbal_behaviour.c/h
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *  V2.0.0     Jan-26-2021     YW              1. modify to fit this project
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//at the beginning of task ,wait a period
#define CHASSIS_TASK_INIT_TIME 357

//the remote controller channel index of controlling vertical speed
#define CHASSIS_X_CHANNEL 1
//the remote controller channel index of controlling horizontal speed
#define CHASSIS_Y_CHANNEL 0

//in some mode, we can use remote controller to control its rotation speed
#define CHASSIS_WZ_CHANNEL 4
//the remote controller switch index of choosing chassis behavior mode
#define CHASSIS_MODE_CHANNEL 0

//ratio that joy stick value (max 660) change to vertical speed (m/s)
#define CHASSIS_VX_RC_SEN 0.006f
//ratio that joy stick value (max 660) change to horizontal speed (m/s)
#define CHASSIS_VY_RC_SEN 0.005f
//in "following yaw angle" mode, the ratio that joy stick value added to chassis angle
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in "not following yaw angle" mode, the ratio that joy stick value changed to chassis rotation speed
#define CHASSIS_WZ_RC_SEN 0.005f

//chassis speed (acceleration) smooth parameter (used in filter)
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//joy stick value dead zone (not become zero at center)
#define CHASSIS_RC_DEADLINE 10

//the ratio used to change four motor speeds into chassis speed
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

//the distance between motor and center of robot
#define MOTOR_DISTANCE_TO_CENTER 0.2f

//chassis task control time interval 2ms    //
#define CHASSIS_CONTROL_TIME_MS 2           //<-These two perhaps can be combined?
//chassis task control time interval 0.002s //
#define CHASSIS_CONTROL_TIME 0.002f         //
//chassis task control frequence, no use now
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//m3508 max can-sent control currentֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//the key (from keyboard) let chassis swing (CTRL)
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassis forward, back, left, right key (from keyboard)
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//ratio of m3508 speed (rpm) changing to chassis speed (m/s),
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//chassis left or right max speed
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.5f

//used in Mecanum wheel speed calculation (wz part)
//This value should be in range [0, 1)
#define CHASSIS_WZ_SET_SCALE 0.1f

//swing max angle when chassis is not set to move, unit: rad
#define SWING_NO_MOVE_ANGLE 0.7f
//swing max angle when chassis is set to move, unit: rad
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //chassis will follow the relative angle between gimbal and chassis
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //chassis will have yaw angle (chassis_yaw) close-looped control
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control
  CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus directly.

} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float32_t accel;   //useless now
  float32_t speed;
  float32_t speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
    //the pointer to remote control
    const RC_ctrl_t *chassis_RC;

    //(disabled "gimbal_task")
    //chassis will use the relative angle of yaw gimbal motor to calculate chassis' euler angle
    const gimbal_motor_t *chassis_yaw_motor;
    //chassis will use the relative angle of pitch gimbal motor to calculate chassis' euler angle
    const gimbal_motor_t *chassis_pitch_motor;

    //(disabled "INS_task")
    //the pointer to the euler angle of gyroscope sensor
    const float32_t *chassis_INS_angle;

    chassis_mode_e chassis_mode;                //chassis control mode
    chassis_mode_e last_chassis_mode;           //last chassis control mode
    chassis_motor_t motor_chassis[4];           //chassis motor data
    pid_type_def motor_speed_pid[4];            //chassis motor speed PID
    pid_type_def chassis_angle_pid;             //chassis follow angle PID

    first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-pointֵ
    first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point

    float32_t vx;                          //chassis vertical speed, positive -> forward, unit: m/s
    float32_t vy;                          //chassis horizontal speed, positive -> left, unit: m/s
    float32_t wz;                          //chassis rotation speed, positive -> counter-clockwise, unit: rad/s
    float32_t vx_set;                      //chassis set vertical speed, positive -> forward, unit: m/s
    float32_t vy_set;                      //chassis set horizontal speed, positive -> left, unit: m/s
    float32_t wz_set;                      //chassis set rotation speed, positive -> counter-clockwise, unit: rad/s
    float32_t chassis_relative_angle;      //the relative angle between chassis and gimbal, unit: rad
    float32_t chassis_relative_angle_set;  //the set relative angle between chassis and gimbal, unit: rad
    float32_t chassis_yaw_set;

    float32_t vx_max_speed;  //max forward speed, unit m/s
    float32_t vx_min_speed;  //max backward speed, unit m/s
    float32_t vy_max_speed;  //max left speed, unit m/s
    float32_t vy_min_speed;  //max right speed, unit m/s
    float32_t chassis_yaw;   //the chassis yaw angle calculated by gyroscope sensor and gimbal motor
    float32_t chassis_pitch; //the chassis pitch angle calculated by gyroscope sensor and gimbal motor
    float32_t chassis_roll;  //the chassis roll angle calculated by gyroscope sensor and gimbal motor

} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
extern void chassis_task(void *pvParameters);

/**
  * @brief          according to the channel value of remote controller, calculate
  *                 chassis vertical and horizontal speed set-point
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" pointer
  * @retval         none
  */
extern void chassis_rc_to_control_vector(float32_t *vx_set, float32_t *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
