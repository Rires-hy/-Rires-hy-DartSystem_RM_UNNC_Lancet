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
  *  V2.0.0		Jan-26-2021		YW				1. modify to fit the project
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

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "gimbal_behaviour.h"

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis control mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_zero_force_control(float32_t *vx_can_set, float32_t *vy_can_set, float32_t *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_no_move_control(float32_t *vx_set, float32_t *vy_set, float32_t *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_infantry_follow_gimbal_yaw_control(float32_t *vx_set, float32_t *vy_set, float32_t *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_engineer_follow_chassis_yaw_control(float32_t *vx_set, float32_t *vy_set, float32_t *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_no_follow_yaw_control(float32_t *vx_set, float32_t *vy_set, float32_t *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_open_set_control(float32_t *vx_set, float32_t *vy_set, float32_t *wz_set, chassis_move_t *chassis_move_rc_to_vector);





//Attention, this is the variable determines chassis behavior mode
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;


/**
  * @brief          assign "chassis_behaviour_mode" variable to which mode
  *                 based on chassis data
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }


    //Set chassis behavior mode based on remote controller
    /*
     * "chassis_behaviour_mode" can be assigned to following mode:
     * ->CHASSIS_ZERO_FORCE
     * ->CHASSIS_NO_MOVE
     * ->CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW
     * ->CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW
     * ->CHASSIS_NO_FOLLOW_YAW
     * ->CHASSIS_OPEN
     */
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }

    //when gimbal at some modes, such as initialization, chassis do not move
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }


    //add your own logic to enter the new mode
    //customized by user


    //according to behavior mode, choose a chassis control mode
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
}


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
void chassis_behaviour_control_set(float32_t *vx_set, float32_t *vy_set, float32_t *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          when chassis behavior mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is CHASSIS_VECTOR_RAW. In this mode, set value
  *                 will be sent to CAN bus directly. Thus this function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus directly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus directly.
  * @param[out]     wz_can_set: wz rotation speed value, it will be sent to CAN bus directly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_zero_force_control(float32_t *vx_can_set, float32_t *vy_can_set, float32_t *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
  * @brief          when chassis behavior mode is CHASSIS_NO_MOVE, chassis control mode
  *                 is speed control mode (CHASSIS_VECTOR_NO_FOLLOW_YAW). chassis does
  *                 not follow gimbal, and the function will set all speed zero to make
  *                 chassis do not move.
  * @param[out]     vx_set: vx speed value, positive -> forward, negative -> backward
  * @param[out]     vy_set: vy speed value, positive -> left, negative -> right
  * @param[out]     wz_set: wz rotation speed value, positive -> counter-clockwise, negative -> clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_no_move_control(float32_t *vx_set, float32_t *vy_set, float32_t *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is
  * 				speed control mode (CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW). In this mode, chassis will follow
  * 				gimbal angle. Then, chassis rotation speed can be calculated from the angle difference
  *					between gimbal and chassis. It also provide swing function.
  * @param[out]     vx_set: vx speed value, positive -> forward, negative -> backward
  * @param[out]     vy_set: vy speed value, positive -> left, negative -> right
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_infantry_follow_gimbal_yaw_control(float32_t *vx_set, float32_t *vy_set, float32_t *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //get x/y speed in general condition
    //based on remote controller channels and keyboards
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //following steps determines "angle_set" (angle difference between gimbal and chassis)

    //"swing_angle" is generated by sin function
    static float32_t swing_angle = 0.0f;

    //"swing_time" is the input time of sin
    static float32_t swing_time = 0.0f;
    
    //"max_angle" is the max angle that chassis will rotate
    static float32_t max_angle = SWING_NO_MOVE_ANGLE;

    //"swing_time" plus(+) "add_time" in one control cycle
    static float32_t const add_time = PI * 0.5f * configTICK_RATE_HZ / CHASSIS_CONTROL_TIME_MS;
    
    //"swing_flag" determines if swing
    static uint8_t swing_flag = 0;

    //check if swing based on keyboard swing key input, and set "swing_flag"
    //here chassis swings only when key is pressed
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }

    //judge if keyboard is controlling the chassis. if yes, reduce the "max_angle"
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
    
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }

    //swing_time  range [0, 2*PI]
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }

    *angle_set = swing_angle;
}

/**
  * @brief          when chassis behavior mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis
  *					control mode is speed control mode (CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW).
  *                 In this mode, chassis will follow chassis yaw angle, chassis rotation speed
  *                 is calculated from the angle difference between set angle and chassis yaw angle.
  * @param[out]     vx_set: vx speed value, positive -> forward, negative -> backward
  * @param[out]     vy_set: vy speed value, positive -> left, negative -> right
  * @param[out]     angle_set: control angle, range: [-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_engineer_follow_chassis_yaw_control(float32_t *vx_set, float32_t *vy_set, float32_t *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
  * @brief          when chassis behavior mode is CHASSIS_NO_FOLLOW_YAW, chassis
  * 				control mode is speed control mode (CHASSIS_VECTOR_NO_FOLLOW_YAW).
  *                 chassis will not follow angle, chassis rotation speed is set by
  *                 "wz_set" directly.
  * @param[out]     vx_set: vx speed value, positive -> forward, negative -> backward
  * @param[out]     vy_set: vy speed value, positive -> left, negative -> right
  * @param[out]     wz_set: rotation speed value, positive -> counter-clockwise, negative -> clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_no_follow_yaw_control(float32_t *vx_set, float32_t *vy_set, float32_t *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode
  *					is raw control mode (CHASSIS_VECTOR_RAW). All x/y/wz set value
  *					will be sent to can bus from raw remote controller values.
  * @param[out]     vx_set: vx speed value, positive -> forward, negative -> backward
  * @param[out]     vy_set: vy speed value, positive -> left, negative -> right
  * @param[out]     wz_set: rotation speed value, positive -> counter-clockwise, negative -> clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
static void chassis_open_set_control(float32_t *vx_set, float32_t *vy_set, float32_t *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}
