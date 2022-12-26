/*
 * bsp_can.h
 *
 *  Created on: Oct 21, 2020
 *      Author: LXY
 */

#ifndef INC_BSP_CAN_H_
#define INC_BSP_CAN_H_

#include "main.h"


#define FEEDBACK_ID_BASE      0x205
#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define MOTOR_MAX_NUM         8
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204

//typedef struct
//{
//    uint16_t can_id;
//    int16_t  set_voltage;
//    uint16_t rotor_angle;
//    int16_t  rotor_speed;
//    int16_t  torque_current;
//    uint8_t  temp;
//}moto_info_t;

typedef struct
{
    uint16_t can_id;
    int16_t  set_voltage;
    uint16_t rotor_angle;
    int16_t  rotor_speed;
    int16_t  torque_current;
    uint8_t  temp;
} motor_measure_t;



void can_user_init(CAN_HandleTypeDef* hcan);
void can2_user_init(CAN_HandleTypeDef* hcan);
void set_motor_voltage_CAN1(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_voltage_CAN2(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_voltage_CAN1_1(int16_t v1, int16_t v2, int16_t v3, int16_t v4);


extern motor_measure_t motor_info[MOTOR_MAX_NUM];
#endif /* INC_BSP_CAN_H_ */
