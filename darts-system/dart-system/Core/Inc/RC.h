/*
 * RC.h
 *
 *  Created on: Oct 21, 2020
 *      Author: LXY
 */

#ifndef INC_RC_H_
#define INC_RC_H_

#include "stdio.h"
#include "stdint.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN              ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET           ((uint16_t)1024)
#define RC_CH_VALUE_MAX              ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                     ((uint16_t)1)
#define RC_SW_MID                    ((uint16_t)3)
#define RC_SW_DOWN                   ((uint16_t)2)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W         ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S         ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A         ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D         ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q         ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E         ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT     ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL      ((uint16_t)0x01<<7)


/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
		struct
		{
				uint16_t rx;
				uint16_t ry;
				uint16_t lx;
				uint16_t ly;
				uint8_t  s1;
				uint8_t  s2;
		}rc;

		struct
		{
				int16_t x;
				int16_t y;
				int16_t z;
				uint8_t press_l;
				uint8_t press_r;
		}mouse;

    struct
		{
				uint16_t v;
		}key;

}RC_Ctl_t;

typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;


/* ----------------------- Internal Data ----------------------------------- */
//extern volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];  //double sbus rx buffer to save data

void RemoteDataProcess(uint8_t *pData,RC_Ctl_t RC_CtrlData);
void RemoteDataShow(void) ;

void remote_control_init(void);
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
extern RC_ctrl_t rc_ctrl;
extern RC_Ctl_t RC_CtrlData;
extern int IRQ_flag;
#endif
