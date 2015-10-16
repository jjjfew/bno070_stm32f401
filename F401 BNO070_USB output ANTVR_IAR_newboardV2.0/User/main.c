/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 02/12/2014 17:28:15
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"  
/* Private variables ---------------------------------------------------------*/
#include "stm32f4xx_hal_uart.h"
/* USER CODE BEGIN 0 */
#include "bno070.h"
#include "usb_device.h"
#include <math.h>
#include "oula.h"
#include "key.h"
#include "adc.h"
#include "stmflash.h"    
/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void StartThread(void const * argument);
static void StartThread_joystick(void const * argument);
static void MX_GPIO_Init(void);
extern UART_HandleTypeDef huart2;
extern USBD_HandleTypeDef hUsbDeviceFS;
int8_t test_pbuffer[64];
union keynumT
{                   /*定义一个联合*/
       int8_t key_num_buffer[34];
        struct
        {             /*在联合中定义一个结构*/
                int8_t ID0;
                int8_t ID1;          
                int8_t Q1_1;
                int8_t Q1_2;
                int8_t Q1_3;
                int8_t Q1_4; 
                int8_t Q2_1;
                int8_t Q2_2;
                int8_t Q2_3;
                int8_t Q2_4;   
                int8_t Q3_1;
                int8_t Q3_2;
                int8_t Q3_3;
                int8_t Q3_4; 
                int8_t Q4_1;
                int8_t Q4_2;
                int8_t Q4_3;
                int8_t Q4_4;                   
        }key_num_struct;
}keynum;	


int main(void)
{  
  keynum.key_num_struct.ID0=0x11;
  keynum.key_num_struct.ID1=0x22;
  keynum.key_num_struct.Q1_1=0x01;             //0x01000021 CTRL
  keynum.key_num_struct.Q1_2=0x00;
  keynum.key_num_struct.Q1_3=0x00;
  keynum.key_num_struct.Q1_4=0x21;
  keynum.key_num_struct.Q2_1=0x00;             //0x41 A
  keynum.key_num_struct.Q2_2=0x00;  
  keynum.key_num_struct.Q2_3=0x00;
  keynum.key_num_struct.Q2_4=0x41;   
  keynum.key_num_struct.Q3_1=0x00;              //0X42 B
  keynum.key_num_struct.Q3_2=0x00;
  keynum.key_num_struct.Q3_3=0x00;
  keynum.key_num_struct.Q3_4=0x42;
  keynum.key_num_struct.Q4_1=0x00;              //0X43 C
  keynum.key_num_struct.Q4_2=0x00;  
  keynum.key_num_struct.Q4_3=0x00;
  keynum.key_num_struct.Q4_4=0x43;     
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  HAL_Uart_MspInit();
  /* Configure the system clock */
  SystemClock_Config();
  
  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();    
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  KEY_Init();
  //Adc_Init();
  bno070_Init();
  
  /* USER CODE END 2 */

  /* Code generated for FreeRTOS */
  /* Create Start thread */

  osThreadDef(USER_Thread, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(USER_Thread), NULL);
  
  osThreadDef(JOYSTICK_Thread, StartThread_joystick, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(JOYSTICK_Thread), NULL);
  /* Start scheduler */
  osKernelStart(NULL, NULL);
  /* We should never get here as control is now taken by the scheduler */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {

  }
  /* USER CODE END 3 */
}

/** System Clock Configuration
*/
#if 1
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336; 
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
#endif
#if 0
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
#endif
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
#if 0
  /*Configure GPIO pins Serial Tx/Rx: PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

}

/* USER CODE BEGIN 4 */
#include "bno070.h"
#include <stdio.h>

static void readProductId()
{
    sensorhub_ProductID_t id;
    int rc = sensorhub_getProductID(&sensorhub, &id);
    if (rc != SENSORHUB_STATUS_SUCCESS) {
      printf("readProductId received error: %d\n", rc);
      return;
    }

    printf("  Version %d.%d\n", id.swVersionMajor, id.swVersionMinor);
    printf("  Part number: %d\n", id.swPartNumber);
    printf("  Build number: %d\n", id.swBuildNumber);
    printf("  Patch number: %d\n\n", id.swVersionPatch);
}


/* USER CODE END 4 */

static int32_t round32(float f) {
	return (int32_t)(f + ((f >= 0) ? 0.5 : -0.5));
}

#define toFixed32(x, Q) round32(x * (float)(1ull << Q))

static void StartThread(void const * argument) {
  while(1){}
#if 0
  /* USER CODE BEGIN 5 */
  
  // ARVR FRS Record.  
  // Parameters can be tuned to suit application.
  const uint32_t arvrFrsData[] = {
    toFixed32(0.2, 30),    // Scaling, dimensionless, 32Q30 format (0.2)
    toFixed32(0.131, 29),  // Max Rotation, radians, 32Q29 format (7.5 degrees)
    toFixed32(0.262, 29),  // Max Error, radians, 32Q29 format (15 degrees)
    toFixed32(0.00175, 29) // Stability magnitude, radians, 32Q29 format (0.1 deg)
  };

  int status;
  printf("Probing for a BNO070...\n");
  sensorhub_probe(&sensorhub);
  
  printf("Requesting product ID...\n");
  readProductId();
  
  printf("Start FRS write...\n");      
  // Note: FRS records are stored in non-volatile memory
  // set ARVR CONFIG FRS Record
  status = sensorhub_writeFRS(&sensorhub, SENSORHUB_FRS_ARVR_CONFIG,
                              arvrFrsData, sizeof(arvrFrsData)/sizeof(uint32_t));
  if (status == 0) {
    printf("ARVR_CONFIG FRS written successfully.\n");
  }
  else {
    printf("ARVR_CONFIG FRS write error: %d\n", status);
  }
  
  // set ARVR GAME CONFIG FRS Record
  status = sensorhub_writeFRS(&sensorhub, SENSORHUB_FRS_ARVR_GAME_CONFIG,
                              arvrFrsData, sizeof(arvrFrsData)/sizeof(uint32_t));
  if (status == 0) {
    printf("ARVR_GAME_CONFIG FRS written successfully.\n");
  }
  else {
    printf("ARVR_GAME_CONFIG FRS write error: %d\n\n", status);
  }
  
  //printf("%08x\n",0.131);
  
  printf("Enabling rotation vector events...\n\n");
    sensorhub_SensorFeature_t settings;
    settings.changeSensitivityEnabled = false;
    settings.wakeupEnabled = false;
    settings.changeSensitivityRelative = false;
    settings.changeSensitivity = 0;              //all reports to be sent
    settings.reportInterval = 1000;             //50ms
    settings.batchInterval = 0;

    sensorhub_setDynamicFeature(&sensorhub, SENSORHUB_ROTATION_VECTOR,&settings);
   // sensorhub_setDynamicFeature(&sensorhub, SENSORHUB_ACCELEROMETER,&settings);
    //sensorhub_setDynamicFeature(&sensorhub, SENSORHUB_GYROSCOPE_CALIBRATED,&settings);
    //sensorhub_setDynamicFeature(&sensorhub, SENSORHUB_MAGNETIC_FIELD_CALIBRATED,&settings);
    
    osDelay(1);               // Only here to make it easier to view on the logic analyzer
    sensorhub_getDynamicFeature(&sensorhub, SENSORHUB_ACCELEROMETER,&settings);
    // sensorhub_getDynamicFeature(&sensorhub, SENSORHUB_RAW_ACCELEROMETER,&settings);

    int reports = 0;
    printf("\nWaiting for events...\n");
    int8_t LED_STAT=0; 
    /* Infinite loop */
    for (;;) {
        sensorhub_Event_t events[5];
        int numEvents = 0;
        int i;

       while (numEvents == 0) {
            sensorhub_poll(&sensorhub, events, 1, &numEvents);   //读5次,把数据放到event中
        }

        for (i = 0; i < numEvents; i++) {
          if ((reports % 100) == 0) {
            
         //   printf("\nEvent %d (I2C transfers %d, retries %d, errors %d)\n", reports, sensorhub.stats->i2cTransfers,sensorhub.stats->i2cRetries, sensorhub.stats->i2cErrors);
         // }
         // if ((reports % 10) == 0) {   //每20个数据print一次
            HAL_GPIO_WritePin( GPIOA, GPIO_PIN_2, LED_STAT);
            LED_STAT=~LED_STAT;
         }
             printEvent(&events[i]);
            reports++;
        }
    }

  /* USER CODE END 5 */ 
#endif
}
 
static void StartThread_joystick(void const * argument)
{
int8_t test_buf[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int8_t keysta,key,adcsta;
int16_t adc_print;
int8_t M1=1,M2=2,M3=3,M4=4;
int8_t get_bufs[64];
int8_t i;


STMFLASH_Read(0X0800C004,test_pbuffer,18);
for(i=0;i<18;i++)
printf("%x ",test_pbuffer[i]);
printf("\n");

if((test_pbuffer[0]!=0x11) || (test_pbuffer[1]!=0x22))   //第一次为空时写入初始的键值
STMFLASH_Write(0X0800C004,keynum.key_num_buffer,18);	

STMFLASH_Read(0X0800C004,test_pbuffer,18);
for(i=0;i<18;i++)
printf("%x ",test_pbuffer[i]);
printf("\n");

int32_t intemp=(test_pbuffer[2]<<24) + (test_pbuffer[3]<<16) + (test_pbuffer[4]<<8) + test_pbuffer[5];
//printf("%x \n",intemp);

while(1)  
 {
#if 0
   USBD_LL_PrepareReceive(&hUsbDeviceFS, 0x01,get_bufs,64);  //从usb读64个字节
/******************************************发送原始的键值2bit**********************************************************/      
   if((get_bufs[0]==0x11) && (get_bufs[1]==0x22))  
   {
   STMFLASH_Read(0X0800C004,keynum.key_num_buffer,18); //从flash中读键值  
   USBD_HID_SendReport(&hUsbDeviceFS,keynum.key_num_buffer,16);   //发送原始键值  
   }
/******************************************收到修改的键值7bit**********************************************************/   
   if((get_bufs[0]==0x22) && (get_bufs[1]==0x11))  
   {  
     STMFLASH_Read(0X0800C004,keynum.key_num_buffer,18); //从flash中读键值   
    int32_t modify_value=(get_bufs[3]<<24) + (get_bufs[4]<<16) + (get_bufs[5]<<8) + get_bufs[6];

    if(get_bufs[2]==1)
    {
      keynum.key_num_struct.Q1_1=get_bufs[3];             //0x01000021 CTRL
      keynum.key_num_struct.Q1_2=get_bufs[4];
      keynum.key_num_struct.Q1_3=get_bufs[5];
      keynum.key_num_struct.Q1_4=get_bufs[6];
    }else if(get_bufs[2]==2)
    {
      keynum.key_num_struct.Q2_1=get_bufs[3];             //0x01000021 CTRL
      keynum.key_num_struct.Q2_2=get_bufs[4];
      keynum.key_num_struct.Q2_3=get_bufs[5];
      keynum.key_num_struct.Q2_4=get_bufs[6];
     }else if(get_bufs[2]==3)
    {
      keynum.key_num_struct.Q3_1=get_bufs[3];             //0x01000021 CTRL
      keynum.key_num_struct.Q3_2=get_bufs[4];
      keynum.key_num_struct.Q3_3=get_bufs[5];
      keynum.key_num_struct.Q3_4=get_bufs[6];
     }else if(get_bufs[2]==4)
    {
      keynum.key_num_struct.Q4_1=get_bufs[3];             //0x01000021 CTRL
      keynum.key_num_struct.Q4_2=get_bufs[4];
      keynum.key_num_struct.Q4_3=get_bufs[5];
      keynum.key_num_struct.Q4_4=get_bufs[6];
     }
    STMFLASH_Write(0X0800C004,keynum.key_num_buffer,18);

   STMFLASH_Read(0X0800C004,keynum.key_num_buffer,18); //从flash中读键值 ,不加的话有时再插上没有保存值
   //USBD_HID_SendReport(&hUsbDeviceFS,keynum.key_num_buffer,16);   //发送新键值，以确保当按下send时正确     
   }
/****************************************************************************************************/      
#endif     
   test_buf[1]++;
   if(test_buf[1]>200)test_buf[1]=0;
   //USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);
   
   key=KEY_Scan(1);//支持连按
   if(key)
    {						    	 
 			 if(key==KEY0_PRESSED)	{test_buf[3]=M1;  USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);keysta=1;}
			 if(key==KEY1_PRESSED)  {test_buf[3]=M2; USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);keysta=1;}
                         if(key==KEY2_PRESSED)	{test_buf[3]=M3;  USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);keysta=1;}
			 if(key==KEY3_PRESSED)  {test_buf[3]=M4; USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);keysta=1;}
                         if(key==KEY4_PRESSED)	{test_buf[3]=5;  USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);keysta=1;}
			 if(key==KEY5_PRESSED)  {test_buf[3]=6; USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);keysta=1;}
                         if(key==KEY6_PRESSED)	{test_buf[3]=7;  USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);keysta=1;}
			 if(key==KEY7_PRESSED)  {test_buf[3]=8; USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);keysta=1;}
                         osDelay(5);
		}else if(keysta)//之前有按下
		{
			keysta=0; //一开始，没按下时，定义的全局变量keysta为0，会发送下面的空数据
                        test_buf[3]=0;
			USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16); //发送松开的命令
		} 
 
#if 0  
   adc_print=Get_Adc2();   
  // printf("adc value: %d\n",adc_print);
   if(adc_print>2600){test_buf[2]=1;USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);adcsta=1; }
   else if(adc_print<1800){test_buf[2]=2;USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);adcsta=1;}
   else 
   {
     osDelay(5);
     if(adcsta)
     {
       test_buf[2]=0;
       USBD_HID_SendReport(&hUsbDeviceFS,test_buf,16);
       adcsta=0;
     }
   }
#endif   
     //printf("adc value: %d\n",adc_print);
 }

}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
