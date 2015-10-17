#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "stm32f4xx_hal_uart.h"

UART_HandleTypeDef huart2;

void HAL_Uart_MspInit(void)
{
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
 if(huart2.Instance==USART1)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    // __GPIOA_CLK_ENABLE();
    __USART1_CLK_ENABLE();
    /**USART2 GPIO Configuration   
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;   //PULLUP??
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW; // LOW
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;  //7
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

 // if(huart2->Instance==USART2)
 // {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __USART1_CLK_DISABLE();
  
    /**USART2 GPIO Configuration   
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
  //}
}

void MX_USART2_UART_Init(void)
{
  // HAL_MspInit();
  //HAL_UART_MspInit(&huart2);
  /*初始化串口2*/  
  huart2.Instance = USART1;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}
#if 1
/*串口重定义*/
int fputc(int ch, FILE *f)
{        
       uint8_t mych[1];
        mych[0]=(uint8_t)ch;
                while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_RESET);  //BUSY_TX,可以不加
                HAL_UART_Transmit(&huart2, mych, 1,10);        //10 times tick
                return ch;
}

int fgetc(FILE *f)
{
    uint8_t mych[1];
    while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_RESET);
    HAL_UART_Receive(&huart2, mych,1,10);
    while (HAL_UART_GetState(&huart2) == HAL_UART_STATE_RESET);
    HAL_UART_Transmit(&huart2, mych, 1,10);
    return mych[0];
}
#endif