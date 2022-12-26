/*
 * bsp_key.c
 *
 *  Created on: Oct 21, 2020
 *      Author: LXY
 */

#include "bsp_key.h"
#include "bsp_led.h"

uint8_t key_scan(void)
{
  static uint8_t key_last = 0;
  static uint8_t key = 0;

  key_last = key;
  key = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);
  if( (key == GPIO_PIN_RESET) && (key_last == GPIO_PIN_SET))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
