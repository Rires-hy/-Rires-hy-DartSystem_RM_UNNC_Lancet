#include "sdcard_task.h"

#include "main.h"

#include "cmsis_os.h"


void sdcard_task(void *pvParameters)
{
    //wait a period for gyroscope task update gyroscope values
//    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    while (1)
    {

    }
}

/**
  * @brief          return yaw motor data pointer
  * @param[in]      none
  * @retval         yaw motor data pointer
  */
