/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to the value of remote controller,
  *             change chassis's behavior.
  * @note       Remember to add gimbal libs in future
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *  V2.0.0     Jan-26-2021     YW              1. modify to fit the project
  *
  @verbatim
  ==============================================================================
    how to add a new chassis behavior mode
    1. in chassis_behaviour.h , add a new behavior name in chassis_behaviour

            typedef enum
            {
                ...
                ...
                CHASSIS_XXX_XXX, // new add
            }chassis_behaviour_e,

    2. implement new function:

        chassis_xxx_xxx_control(float32_t *vx, float32_t *vy, float32_t *wz, chassis_move_t * chassis )

        "vx, vy, wz" param is chassis movement control input.
        'vx': usually means vertical speed, positive -> forward, negative -> backward.
        'vy': usually means horizotal speed, positive -> left, negative -> right.
        'wz': can be rotation speed set or angle set.

        in this new function, you can assign speed to "vx","vy",and "wz",as your wish

    3.  in "chassis_behaviour_mode_set" function, add new logical judgement
        to assign CHASSIS_XXX_XXX to "chassis_behaviour_mode" variable.
        and in the last of the function, add:

            else if (chassis_behaviour_mode == CHASSIS_XXX_XXX)
            {
                chassis_move_mode->chassis_mode = CHASSIS_VECTOR_XXX;
            }

        , where CHASSIS_VECTOR_XXX is one of following four chassis control mode:

        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW:
            ->'vx' and 'vy' are speed control,
            ->'wz' is angle set to control relative angle between chassis and gimbal.
            you can name third parameter to 'xxx_angle_set' other than 'wz'.
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW:
            ->'vx' and 'vy' are speed control,
            ->'wz' is angle set to control absolute angle calculated by gyroscope
            you can name third parameter to 'xxx_angle_set.
        CHASSIS_VECTOR_NO_FOLLOW_YAW:
            ->'vx' and 'vy' are speed control,
            ->'wz' is rotation speed control.
        CHASSIS_VECTOR_RAW:
            ->will use 'vx' 'vy' and 'wz'  to linearly calculate four wheel current set
            from remote controller value directly. current set will be derectly sent to
            can bus.

    4. in the last of "chassis_behaviour_control_set" function, add

            else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
            {
                chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
            }

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "arm_math.h"
#include "chassis_task.h"

typedef enum
{
  CHASSIS_ZERO_FORCE,                   //chassis behaves like no power

  CHASSIS_NO_MOVE,                      //chassis remains stopping

  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,   //chassis follows gimbal, usually used in infantry robot

  CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  /*chassis follows chassis yaw angle, usually used in engineer robot,
                                          because chassis does not have gyroscope, its yaw angle is calculated
                                          by gyroscope value in gimbal and gimbal motor angle, if you have a
                                          gyroscope in chassis, please update yaw, pitch, roll angle in
                                          "chassis_feedback_update" function.*/

  CHASSIS_NO_FOLLOW_YAW,                //chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed

  CHASSIS_OPEN                          //the value of remote controller will multiply a ratio
                                        //get current values and sent to can bus directly
} chassis_behaviour_e;

//in CHASSIS_OPEN mode, values from remote controller will multiply this scale ratio
//get current values and sent to can bus directly.
#define CHASSIS_OPEN_RC_SCALE 10



/**
  * @brief          assign "chassis_behaviour_mode" variable to which mode
  *                 based on chassis data
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
  * @brief          set control value. according to different control mode,
  *                 three input direction values will control corresponding
  *                 movement. this function will call different control function.
  * @param[out]     vx_set: usually controls vertical speed.
  * @param[out]     vy_set: usually controls horizotal speed.
  * @param[out]     wz_set: usually controls rotation speed.
  * @param[in]      chassis_move_rc_to_vector: has all data of chassis
  * @retval         none
  */
extern void chassis_behaviour_control_set(float32_t *vx_set, float32_t *vy_set, float32_t *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
