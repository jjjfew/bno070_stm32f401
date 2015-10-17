#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "key.h"

//按键初始化函数
void KEY_Init(void) //IO初始化
{ 
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pin LED: PA1,PA2,PA3,PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  
  
  
}
//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，KEY0按下
//2，KEY1按下
//3，KEY2按下 
//4，KEY3按下 
//注意此函数有响应优先级,KEY0>KEY1>KEY2>KEY3!!
int8_t KEY_Scan(int8_t mode)
{	 
	static int8_t key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(KEY0==0||KEY1==0||KEY2==0||KEY3==0||KEY4==0||KEY5==0||KEY6==0||KEY7==0))
	{
               // osDelay(5);
		key_up=0;
		if(KEY0==0)return KEY0_PRESSED;
		else if(KEY1==0)return KEY1_PRESSED;
		else if(KEY2==0)return KEY2_PRESSED;
		else if(KEY3==0)return KEY3_PRESSED;
                else if(KEY4==0)return KEY4_PRESSED;
		else if(KEY5==0)return KEY5_PRESSED;
		else if(KEY6==0)return KEY6_PRESSED;
		else if(KEY7==0)return KEY7_PRESSED;                
	}else if(KEY0==1||KEY1==1||KEY2==1||KEY3==1||KEY4==1||KEY5==1||KEY6==1||KEY7==1)key_up=1; 	    
 	return 0;// 无按键按下
}
