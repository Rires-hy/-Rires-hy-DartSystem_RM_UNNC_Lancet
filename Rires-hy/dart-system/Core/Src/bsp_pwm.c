/*
 * bsp_pwm.c
 *
 *  Created on: Oct 21, 2020
 *      Author: LXY
 */

#include "bsp_pwm.h"

/**
  * @brief  start pwm output
  * @param  None
  * @retval None
  */
void pwm_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // start pwm output
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}
