/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      shoot function
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Feb-05-2021     YW              1. modify to fit the project
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"



//remote controller shoot-control switch channel index
#define SHOOT_RC_MODE_CHANNEL       1

//the period between each two times in friction motor ramp filter, do not change
#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

//the added value each time in friction motor ramp filter
#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//the key that turn on/off friction motor and laser
//(go into SHOOT_READY_FRIC/SHOOT_STOP when switch on remote controller switch is in middle position)
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//after one shoot complete, bullet shot out, check the duration period to avoid macro switch triggered incorrectly
#define SHOOT_DONE_KEY_OFF_TIME     15
//mouse button long press threshold
#define PRESS_LONG_TIME             400
//remote controller switch at down position for a while threshold (go into SHOOT_CONTINUE_BULLET, this mode is used to clear bullet)
#define RC_S_LONG_TIME              2000
//friction motor fast speed duration (after this duration, back to slow speed)
#define UP_ADD_TIME                 80

//push bullet motor feedback encoder value range
#define SHOOT_PUSH_HALF_ECD_RANGE              4096
//push bullet motor feedback encoder value range
#define SHOOT_PUSH_ECD_RANGE                   8192    //YW: it's 8191 originally, but I think it should be 8192

//ratio that change rpm into speed
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
//ratio that change feedback encoder value to angle
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
//used to change lap of rotor into output axle angle (details check .c file)
#define FULL_COUNT                  18

//push bullet motor speed in different mode
#define TRIGGER_SPEED               10.0f
#define CONTINUE_TRIGGER_SPEED      15.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500       //useless

//macro switch state
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//bullet stuck judge speed
#define BLOCK_TRIGGER_SPEED         1.0f
//bullet stuck time threshold
#define BLOCK_TIME                  700
//motor reverse rotate duration each time when bullet stuck
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f       //useless

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//push bullet motor pid
#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

//(used in referee check)
#define SHOOT_HEAT_REMAIN_VALUE     80

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    pid_type_def trigger_motor_pid;
    float32_t trigger_speed_set;                         //push bullet motor set speed used in bullet stuck handler, this value will be sent to "speed_set"
    float32_t speed;                                     //push bullet motor feedback value
    float32_t speed_set;                                 //push bullet motor set speed value used in pid target value
    float32_t angle;
    float32_t set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;


//because shoot and gimbal use the same CAN id, so shoot task is executed in gimbal task
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#endif
