#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "stm32f4xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//STM32内部FLASH读写 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/9
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址

int8_t STMFLASH_ReadWord(int32_t faddr);		  	//读出字  
void STMFLASH_Write(int32_t WriteAddr,int8_t *pBuffer,int32_t NumToWrite);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(int32_t ReadAddr,int8_t *pBuffer,int32_t NumToRead);   		//从指定地址开始读出指定长度的数据
						   
#endif

















