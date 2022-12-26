/*
 * sd_card.h
 *
 *  Created on: Apr 10, 2021
 *      Author: 10024
 */

#ifndef SDCARD_TASK_H_
#define SDCARD_TASK_H_
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

/**
  * @brief          sd card task, osDelay GIMBAL_CONTROL_TIME (1ms)
  * @param[in]      pvParameters: null
  * @retval         none
  */
extern void sdcard_task(void *pvParameters);


#endif /* SDCARD_TASK_H_ */
