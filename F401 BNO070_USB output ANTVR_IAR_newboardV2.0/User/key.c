#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "key.h"

//������ʼ������
void KEY_Init(void) //IO��ʼ��
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
//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��KEY0����
//2��KEY1����
//3��KEY2���� 
//4��KEY3���� 
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>KEY3!!
int8_t KEY_Scan(int8_t mode)
{	 
	static int8_t key_up=1;//�������ɿ���־
	if(mode)key_up=1;  //֧������		  
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
 	return 0;// �ް�������
}
