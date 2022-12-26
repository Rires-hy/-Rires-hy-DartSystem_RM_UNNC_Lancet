/*
 * pid.h
 *
 *  Created on: Oct 21, 2020
 *      Author: LXY
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;

  float ref;      // target value
  float fdb;      // feedback value
  float err[2];   // error and last error

  float p_out;
  float i_out;
  float d_out;
  float output;
}pid_struct_t;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

float  pid_calc(pid_struct_t *pid, float ref, float fdb);

#endif /* INC_PID_H_ */
