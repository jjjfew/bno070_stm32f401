#ifndef __KEY_H
#define __KEY_H	  	 
// 
//#define KEY0 PEin(4)   	//PE4
//#define KEY1 PEin(3)	//PE3 
//#define KEY2 PEin(2)	//PE2
//#define KEY3 PAin(0)	//PA0  WK_UP
 
#define KEY0  HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)//读取按键0
#define KEY1  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)//读取按键1
#define KEY2  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)//读取按键2 
#define KEY3  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)//读取按键3 
#define KEY4  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)//读取按键0
#define KEY5  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)//读取按键1
#define KEY6  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)//读取按键2 
#define KEY7  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)//读取按键3 

#define KEY0_PRESSED 		1
#define KEY1_PRESSED    	2
#define KEY2_PRESSED	        3
#define KEY3_PRESSED	        4
#define KEY4_PRESSED 		5
#define KEY5_PRESSED    	6
#define KEY6_PRESSED	        7
#define KEY7_PRESSED	        8

void KEY_Init(void);//IO初始化
int8_t KEY_Scan(int8_t mode);  	//按键扫描函数					    
#endif