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
#include <math.h>
/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void StartThread(void const * argument);
static void MX_GPIO_Init(void);
extern UART_HandleTypeDef huart2;

int main(void)
{  
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  /* Initialize all configured peripherals */
  MX_GPIO_Init();    
  MX_USART2_UART_Init();
  
  /* USER CODE BEGIN 2 */
  bno070_Init();
  
  /* USER CODE END 2 */

  /* Code generated for FreeRTOS */
  /* Create Start thread */
  osThreadDef(USER_Thread, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(USER_Thread), NULL);

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

void float_char(float f,unsigned char *s)
{
 unsigned char *p;
 
 p = (unsigned char *)&f;

    *s = *p;

    *(s+1) = *(p+1);

    *(s+2) = *(p+2);

    *(s+3) = *(p+3);
}

//*********************************************
//		四元素转欧拉角（单位：弧度）           注意坐标轴方向
//*********************************************
void Quat2Euler(float *q, float *e)       //*q为四元数，*e为欧拉角
{
    float Q[4];
    float T[3][3];
    float Pitch=e[0];  //x
    float Yaw=e[1];     //y
    float Roll=e[2];    //z
      
	Q[0] = q[3];       //w
	Q[1] = q[0]*(-1);  //x 
	Q[2] = q[1]*(-1);  //y
	Q[3] = q[2]*(-1);  //z
    T[0][0] =   Q[0]*Q[0]+(-Q[1])*(-Q[1])-(-Q[2])*(-Q[2])-(-Q[3])*(-Q[3]) ;
    T[0][1] =                    2*((-Q[1])*(-Q[2])-Q[0]*(-Q[3]));
    T[0][2] =                    2*((-Q[1])*(-Q[3])+Q[0]*(-Q[2]));

    T[1][0] =                    2*((-Q[1])*(-Q[2])+Q[0]*(-Q[3]));
    T[1][1] =   Q[0]*Q[0]-(-Q[1])*(-Q[1])+(-Q[2])*(-Q[2])-(-Q[3])*(-Q[3]) ;
    T[1][2] =                    2*((-Q[2])*(-Q[3])-Q[0]*(-Q[1]));

    T[2][0] =                    2*((-Q[1])*(-Q[3])-Q[0]*(-Q[2]));
    T[2][1] =                    2*((-Q[2])*(-Q[3])+Q[0]*(-Q[1]));
    T[2][2] =   Q[0]*Q[0]-(-Q[1])*(-Q[1])-(-Q[2])*(-Q[2])+(-Q[3])*(-Q[3]) ;

    Pitch  = atan( T[2][1]/T[2][2]);  
    Yaw    = asin(-T[2][0]);
    Roll   = atan( T[1][0]/T[0][0]);
 
    if(T[2][2]<0)
    {
        if(Yaw  < 0)
        {
           Yaw = Yaw +3.1416;
        }
        else
        {
           Yaw = Yaw -3.1416;
        }
    }

    if(T[0][0]<0)
    {
        if(T[1][0]>0)
        {
            Roll = Roll + 3.1416;
        }
        else
        {
            Roll = Roll - 3.1416;
        }
    }
    
    e[0]=Pitch;  //x
    e[1]=Yaw;     //y
    e[2]=Roll;
}

void printEvent(const sensorhub_Event_t * event)
{
    float scaleQ14 = 1.0f / (1 << 14);
    float scaleQ8 = 1.0f / (1 << 8);
    float scaleQ9 = 1.0f / (1 << 9);
    float scaleQ5 = 1.0f / (1 << 5);
    float r, i, j, k;
    int16_t x,y,z;
    int8_t xh,xl,yh,yl,zh,zl;
    int8_t msg[40],dat[30];
    msg[0]=0xa5;
    int tmp;
        switch (event->sensor) {
    case SENSORHUB_RAW_ACCELEROMETER:
        printf("Raw acc: %d %d %d\n",
               event->un.rawAccelerometer.x,
               event->un.rawAccelerometer.y, 
               event->un.rawAccelerometer.z);
        break;
        
    case SENSORHUB_ACCELEROMETER:
        x=event->un.accelerometer.x_16Q8;
        y=event->un.accelerometer.y_16Q8;
        z=event->un.accelerometer.z_16Q8;
        xh=(int8_t)(x>>8);xl=(int8_t)x; yh=(int8_t)(y>>8);yl=(int8_t)y; zh=(int8_t)(z>>8);zl=(int8_t)z;
        
   //    float_char((float)(scaleQ8*x),dat);       //x data
       // msg[1]=0xAF;msg[2]=0x04; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];msg[7]=0x07;
      // HAL_UART_Transmit(&huart2, msg, 6,10); 
        
     //  float_char((float)(scaleQ8*y),dat);       //y data
      //   msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];  
        
     //  float_char((float)(scaleQ8*z),dat);       //z data
      //  msg[1]=0xA3; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        
         msg[1]=0xAF; msg[2]=0x1C;
         msg[3]=xh;msg[4]=xl;msg[5]=yh;msg[6]=yl;msg[7]=zh;msg[8]=zl;
         for(tmp=9;tmp<31;tmp++)msg[tmp]=0x22;
         msg[31]=0x1F;
        HAL_UART_Transmit(&huart2, msg, 32,10);  
      // printf("Acc: %5.3f %5.3f %5.3f\n",scaleQ8*event->un.accelerometer.x_16Q8,scaleQ8*event->un.accelerometer.y_16Q8,scaleQ8*event->un.accelerometer.z_16Q8);
        break;
     
     case SENSORHUB_GYROSCOPE_CALIBRATED:
        x=event->un.gyroscope.x_16Q9;
        y=event->un.gyroscope.y_16Q9;
        z=event->un.gyroscope.z_16Q9;
        xh=(int8_t)(x>>8);xl=(int8_t)x; yh=(int8_t)(y>>8);yl=(int8_t)y; zh=(int8_t)(z>>8);zl=(int8_t)z;

       float_char((float)(scaleQ9*x),dat);       //x data
        msg[1]=0xA1;msg[2]=0x04; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];msg[7]=0x07;
       HAL_UART_Transmit(&huart2, msg, 6,10); 
        
       float_char((float)(scaleQ9*y),dat);       //y data
        msg[1]=0xA2; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        HAL_UART_Transmit(&huart2, msg, 6,10);  
        
       float_char((float)(scaleQ9*z),dat);       //z data
        msg[1]=0xA3; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        HAL_UART_Transmit(&huart2, msg, 6,10);  
 
      // printf("Gyrop: %5.3f %5.3f %5.3f\n",scaleQ9*event->un.gyroscope.x_16Q9,scaleQ9*event->un.gyroscope.y_16Q9,scaleQ9*event->un.gyroscope.z_16Q9);
        break;    
        
     case SENSORHUB_MAGNETIC_FIELD_CALIBRATED:
        x=event->un.magneticField.x_16Q4;
        y=event->un.magneticField.y_16Q4;
        z=event->un.magneticField.z_16Q4;
        xh=(int8_t)(x>>8);xl=(int8_t)x; yh=(int8_t)(y>>8);yl=(int8_t)y; zh=(int8_t)(z>>8);zl=(int8_t)z;

       float_char((float)(scaleQ5*x),dat);       //x data
        msg[1]=0xA1;msg[2]=0x04; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];msg[7]=0x07;
       HAL_UART_Transmit(&huart2, msg, 6,10); 
        
       float_char((float)(scaleQ5*y),dat);       //y data
        msg[1]=0xA2; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        HAL_UART_Transmit(&huart2, msg, 6,10);  
        
       float_char((float)(scaleQ5*z),dat);       //z data
        msg[1]=0xA3; msg[3]=dat[3];msg[4]=dat[2];msg[5]=dat[1];msg[6]=dat[0];
        HAL_UART_Transmit(&huart2, msg, 6,10);  
       
        //printf("Magnt: %5.3f %5.3f %5.3f\n",scaleQ5*event->un.magneticField.x_16Q4,scaleQ5*event->un.magneticField.y_16Q4,scaleQ5*event->un.magneticField.z_16Q4);
        break;    
                   
    case SENSORHUB_ROTATION_VECTOR:
        r = scaleQ14 * event->un.rotationVector.real_16Q14;
        i = scaleQ14 * event->un.rotationVector.i_16Q14;
        j = scaleQ14 * event->un.rotationVector.j_16Q14;
        k = scaleQ14 * event->un.rotationVector.k_16Q14;
        
        float Quart[4];float Euler[3];
        Quart[0]=i;Quart[1]=j;Quart[2]=k;Quart[3]=r;

        Quat2Euler(Quart,Euler);   
       // printf("Orientation: r:%5.3f i:%5.3f j:%5.3f k:%5.3f \n", r, i, j, k);
       // printf("Pitch:%5.3f Yaw:%5.3f Roll:%5.3f\n",Euler[0]*57,Euler[1]*57,Euler[2]*57);
        int16_t yaw=Euler[1]*57*10;
        int16_t pitch=Euler[0]*57*10;
        int16_t roll=Euler[2]*57*10;
        int16_t temp=0xaF+2;   //总长度
	int8_t ctemp;
          
        msg[1]=0x5a; msg[2]=16; msg[3]=0xa1;
        if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;             //16bit的int型分两次发，先发高8位，后发低8位
        msg[4]=ctemp;
	temp+=ctemp;
	ctemp=yaw;
        msg[5]=ctemp;
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	msg[6]=ctemp;
	temp+=ctemp;
	ctemp=pitch;
	msg[7]=ctemp;
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	msg[8]=ctemp;
	temp+=ctemp;
	ctemp=roll;
	msg[9]=ctemp;
	temp+=ctemp;
        
        int16_t alt=0x1111;
	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	msg[10]=ctemp;
	temp+=ctemp;
	ctemp=alt;
	msg[11]=ctemp;
	temp+=ctemp;

        int16_t tempr=0x1111;
	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	msg[12]=ctemp;
	temp+=ctemp;
	ctemp=tempr;
	msg[13]=ctemp;
	temp+=ctemp;

        int16_t press=0x1111;
	if(press<0)press=32768-press;
	ctemp=press>>8;
	msg[14]=ctemp;
	temp+=ctemp;
	ctemp=press;
	msg[15]=ctemp;
	temp+=ctemp;
        
        msg[16]=temp%256;
        msg[17]=0xaa;
        
        //HAL_UART_Transmit(&huart2, msg, 32,10);  
        printf("r:%f i:%f j:%f k:%f\n",r,i,j,k);
       // printf("roll:%f pitch:%f yaw:%f\n",Euler[0]*57,Euler[1]*57,Euler[2]*57);
        
        break;
        
    default:
        printf("Unknown sensor: %d\n", event->sensor);
        break;
    }
}
/* USER CODE END 4 */

static int32_t round32(float f) {
	return (int32_t)(f + ((f >= 0) ? 0.5 : -0.5));
}

#define toFixed32(x, Q) round32(x * (float)(1ull << Q))

static void StartThread(void const * argument) {

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
    settings.reportInterval = 50000;             //50ms
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

    /* Infinite loop */
    for (;;) {
        sensorhub_Event_t events[5];
        int numEvents = 0;
        int i;

        while (numEvents == 0) {
            sensorhub_poll(&sensorhub, events, 5, &numEvents);   //读5次,把数据放到event中
        }

        for (i = 0; i < numEvents; i++) {
          if ((reports % 100) == 0) {
            printf("\nEvent %d (I2C transfers %d, retries %d, errors %d)\n", reports, sensorhub.stats->i2cTransfers,sensorhub.stats->i2cRetries, sensorhub.stats->i2cErrors);
          }
         // if ((reports % 10) == 0) {   //每20个数据print一次
            printEvent(&events[i]);
         //}
          reports++;
        }
    }

  /* USER CODE END 5 */ 

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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
