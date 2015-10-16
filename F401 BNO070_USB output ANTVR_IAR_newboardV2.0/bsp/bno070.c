/****************************************************************************
* Copyright (C) 2015 Hillcrest Laboratories, Inc.
*
* Filename:
* Date:
* Description:
*
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use of such information nor for any infringement
* of patents or other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/

#include "bno070.h"
#include "i2c_master_transfer.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>

// extern I2C_HandleTypeDef hi2c1; /* See main.c */

#define BNO070_APP_I2C_8BIT_ADDR (0x48 << 1)
#define BNO070_BOOTLOADER_I2C_8BIT_ADDR (0x28 << 1)

// LED: PA2
#define BNO_LED_PORT GPIOA
#define BNO_LED_BIT GPIO_PIN_2

// DEBUG: PB10
#define BNO_DEBUG_PORT GPIOB
#define BNO_DEBUG_BIT GPIO_PIN_10

// RSTn: PA8
#define BNO_RSTN_PORT GPIOA
#define BNO_RSTN_BIT GPIO_PIN_8

// BOOTn: PB5
#define BNO_BOOTN_PORT GPIOB
#define BNO_BOOTN_BIT GPIO_PIN_5

// INTn: PB1
#define BNO_INTN_PORT GPIOB
#define BNO_INTN_BIT GPIO_PIN_1

// Addr_Sel: PB0
#define BNO_ADR_PORT GPIOB
#define BNO_ADR_BIT GPIO_PIN_0

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
static void bno070_I2C1_Init(void)
{
  /* Initialize I2C peripheral 初始化I2C总线 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;                //400000
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);  //包含初始化PB8,PB9引脚

}

static void bno070_GPIO_Init()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pin LED: D13, Nucleo PA2*/
  GPIO_InitStruct.Pin = BNO_LED_BIT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(BNO_LED_PORT, &GPIO_InitStruct);
  
  /* Turn off LED.  If it comes on, this indicates an error */
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);    // 关闭A10   以夺棱柑柑奇才
  HAL_GPIO_WritePin( BNO_LED_PORT,  BNO_LED_BIT, GPIO_PIN_SET);    // 1代表关闭LED
 
  /*Configure GPIO pin DEBUG : D6, Nucleo PB10 */
  GPIO_InitStruct.Pin = BNO_DEBUG_BIT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(BNO_DEBUG_PORT, &GPIO_InitStruct);
  
  /*Configure GPIO pin RSTn*/
  GPIO_InitStruct.Pin = BNO_RSTN_BIT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(BNO_RSTN_PORT, &GPIO_InitStruct);
  //HAL_GPIO_WritePin(BNO_RSTN_PORT, BNO_RSTN_BIT, 1);

  /*Configure GPIO pin BOOTn: D4, Nucleo PB5 */
  GPIO_InitStruct.Pin = BNO_BOOTN_BIT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(BNO_BOOTN_PORT, &GPIO_InitStruct);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);

  /*Configure GPIO pin INTn: D2, Nucleo PA6*/
  GPIO_InitStruct.Pin = BNO_INTN_BIT;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BNO_INTN_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = BNO_ADR_BIT;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(BNO_ADR_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BNO_ADR_PORT, BNO_ADR_BIT, 0);
}

/* Initialize resources for BNO070 */
void bno070_Init()
{
    bno070_GPIO_Init();
    bno070_I2C1_Init();
}

/* Support functions for BNO070-based sensorhub */
static void debugPrintf(const char *format, ...)
{
  static char buffer[256];

  va_list ap;
  va_start(ap, format);
  vsprintf(buffer, format, ap);
  printf("%s", buffer);
  va_end(ap);
}

static void logError(const struct sensorhub_s *sh, int err)
{
  /* Send this to the debugger's Terminal I/O window */
  printf("BNO070 error detected: %d...\n", err);
  HAL_GPIO_WritePin( BNO_LED_PORT,  BNO_LED_BIT, 0); // Turn on LED
}

static int i2cTransfer(const struct sensorhub_s *sh, 
                       uint8_t address,
                       const uint8_t *sendData, 
                       int sendLength, 
                       uint8_t *receiveData, 
                       int receiveLength)
{
 
  int rc = HAL_I2C_Master_Transfer(&hi2c1, address, (uint8_t *) sendData, sendLength, receiveData, receiveLength, 1000);
  if (rc == HAL_OK)
  {
    return SENSORHUB_STATUS_SUCCESS;
  } 
  else
  { 
    return SENSORHUB_STATUS_ERROR_I2C_IO;
  }
}

static void gpioSetRSTN(const struct sensorhub_s *sh, int value)
{
  HAL_GPIO_WritePin(BNO_RSTN_PORT, BNO_RSTN_BIT, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void gpioSetBOOTN(const struct sensorhub_s *sh, int value)
{
  HAL_GPIO_WritePin(BNO_BOOTN_PORT, BNO_BOOTN_BIT, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int gpioGetHOST_INTN(const struct sensorhub_s *sh)
{
  return HAL_GPIO_ReadPin(BNO_INTN_PORT, BNO_INTN_BIT);
}

static void delay(const struct sensorhub_s *sh, int milliseconds)
{
  osDelay(milliseconds);
}

static uint32_t getTick(const struct sensorhub_s *sh)
{
    return HAL_GetTick();
}

static sensorhub_stats_t sensorhubStats;

// Create sensorhub based on BNO070
sensorhub_t sensorhub = {
    BNO070_APP_I2C_8BIT_ADDR,
    BNO070_BOOTLOADER_I2C_8BIT_ADDR,
    &sensorhubStats,
    i2cTransfer,
    gpioSetRSTN,
    gpioSetBOOTN,
    gpioGetHOST_INTN,
    delay,
    getTick,
    logError,
    0, // debugPrintf,
    5,                          /* I2C retries */
    NULL                        /* cookie */
};

