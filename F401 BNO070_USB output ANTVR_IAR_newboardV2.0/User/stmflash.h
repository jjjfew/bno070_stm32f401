#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "stm32f4xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//STM32�ڲ�FLASH��д ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/9
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
//FLASH��ʼ��ַ
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ

int8_t STMFLASH_ReadWord(int32_t faddr);		  	//������  
void STMFLASH_Write(int32_t WriteAddr,int8_t *pBuffer,int32_t NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(int32_t ReadAddr,int8_t *pBuffer,int32_t NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����
						   
#endif

















