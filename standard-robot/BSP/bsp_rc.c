#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/**
  * @brief          initialize usart1 which is used for remote controller
  * @param[in]      rx1_buf: memory buffer 1
  * @param[in]      rx2_buf: memory buffer 2
  * @param[in]      dma_buf_num: data length
  * @retval         none
  */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    //enable idle interrupt
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

void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart1);
}


void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart1);
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    hdma_usart1_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
    __HAL_UART_ENABLE(&huart1);

}






