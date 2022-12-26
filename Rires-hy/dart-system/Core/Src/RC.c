/*
 * RC.c
 *
 *  Created on: Oct 21, 2020
 *      Author: LXY
 */

#include "RC.h"

/* ----------------------- Data Struct ------------------------------------- */

/* ----------------------- Internal Data ----------------------------------- */
//volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];  //double sbus rx buffer to save data
//remote control data
//Ò£¿ØÆ÷¿ØÖÆ±äÁ¿
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes
//½ÓÊÕÔ­Ê¼Êý¾Ý£¬Îª18¸ö×Ö½Ú£¬¸øÁË36¸ö×Ö½Ú³¤¶È£¬·ÀÖ¹DMA´«ÊäÔ½½ç
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
RC_Ctl_t RC_CtrlData;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
int IRQ_flag=0;

/* ----------------------- Function Implements  ---------------------------- */

/******************************************************************************
* @fn      RC_Init
*
* @brief   configure stm32 usart2 port
*          -   USART Parameters
*              -   100Kbps
*              -   8-N-1
*          -   DMA Mode
*            * @return  None.
*
* @note    This code is fully tested on STM32F405RGT6 Platform, You can port it
*          to the other platform. Using doube buffer to receive data prevent losing data.  */
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
		SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
//
//    /* -------------- Configure DMA -----------------------------------------*/
//	{
//		DMA_InitTypeDef   dma;
//
//    DMA_DeInit(DMA1_Stream5);
//		dma.DMA_Channel              = DMA_Channel_4;
//		dma.DMA_PeripheralBaseAddr   = (uint32_t)&(USART2->DR);
//		dma.DMA_Memory0BaseAddr      = (uint32_t)&sbus_rx_buffer[0][0];
//		dma.DMA_DIR                  = DMA_DIR_PeripheralToMemory;
//		dma.DMA_BufferSize           = RC_FRAME_LENGTH;
//		dma.DMA_PeripheralInc        = DMA_PeripheralInc_Disable;
//		dma.DMA_MemoryInc            = DMA_MemoryInc_Enable;
//		dma.DMA_PeripheralDataSize   = DMA_PeripheralDataSize_Byte;
//		dma.DMA_MemoryDataSize       = DMA_MemoryDataSize_Byte;
//		dma.DMA_Mode                 = DMA_Mode_Circular;
//		dma.DMA_Priority             = DMA_Priority_VeryHigh;
//		dma.DMA_FIFOMode             = DMA_FIFOMode_Disable;
//		dma.DMA_FIFOThreshold        = DMA_FIFOThreshold_1QuarterFull;
//		dma.DMA_MemoryBurst          = DMA_MemoryBurst_Single;
//		dma.DMA_PeripheralBurst      = DMA_PeripheralBurst_Single;
//		DMA_DoubleBufferModeConfig(DMA1_Stream5,(uint32_t)&sbus_rx_buffer[1][0], DMA_Memory_0);   //first used memory configuration
//		DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
//		DMA_Init(DMA1_Stream5,&dma);
//
//    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
//		DMA_Cmd(DMA1_Stream5,ENABLE);
//		}
//}

/******************************************************************************
* @fn      RemoteDataProcess
*
* @brief   resolution rc protocol data.
* @pData   a point to rc receive buffer.
* @return  None.
* @note    RC_CtrlData is a global variable.you can deal with it in other place.
*/
void RemoteDataProcess(uint8_t *pData,RC_Ctl_t RC_CtrlData)
{
	if(pData == NULL)
	{
		return;
	}
	RC_CtrlData.rc.rx = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	RC_CtrlData.rc.ry = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.lx = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ly = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

  RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);

  RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];

	RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
	//your control code ¡­.
}
extern UART_HandleTypeDef huart2;



void RemoteDataShow(void)
{
			printf("%d\r\n",RC_CtrlData.rc.rx);
			printf("%d\r\n",RC_CtrlData.rc.ry);

			printf("%d\r\n",RC_CtrlData.rc.lx);

			printf("%d\r\n",RC_CtrlData.rc.ly);
			printf("%d\r\n",RC_CtrlData.rc.s1);
			printf("%d\r\n",RC_CtrlData.rc.s2);


}
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}



/******************************************************************************
* @fn      USART2_IRQHandler
*
* @brief   USART2 irq, we are care of ilde interrupt that means receiving the one frame datas is finished.
*
* @return  None.
*
* @note    This code is fully tested on STM32F405RGT6 Platform, You can port it
*          to the other platform.
*/
void USART1_IRQHandler(void)
{
		IRQ_flag=1;
    if(huart1.Instance->SR & UART_FLAG_RXNE)//½ÓÊÕµ½Êý¾Ý
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //Ê§Ð§DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //»ñÈ¡½ÓÊÕÊý¾Ý³¤¶È,³¤¶È = Éè¶¨³¤¶È - Ê£Óà³¤¶È
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //ÖØÐÂÉè¶¨Êý¾Ý³¤¶È
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //Éè¶¨»º³åÇø1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //Ê¹ÄÜDMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
//								RemoteDataProcess(sbus_rx_buf[0], RC_CtrlData);
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //Ê§Ð§DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //»ñÈ¡½ÓÊÕÊý¾Ý³¤¶È,³¤¶È = Éè¶¨³¤¶È - Ê£Óà³¤¶È
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //ÖØÐÂÉè¶¨Êý¾Ý³¤¶È
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //Éè¶¨»º³åÇø0
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //Ê¹ÄÜDMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //´¦ÀíÒ£¿ØÆ÷Êý¾Ý
//                RemoteDataProcess(sbus_rx_buf[1], RC_CtrlData);
							sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}
