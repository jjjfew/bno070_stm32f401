#include "stmflash.h"

#define FLASH_WRP_SECTORS (OB_WRP_SECTOR_3 | OB_WRP_SECTOR_4) 

FLASH_OBProgramInitTypeDef OBInit;
uint32_t SectorsWRPStatus = 0xFFF;


//��ȡָ����ַ����(32bit) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
#if 0
int32_t STMFLASH_ReadWord(int32_t faddr)
{
	return *(uint32_t*)faddr; 
}  
#endif

int8_t STMFLASH_ReadWord(int32_t faddr)
{
	return *(uint32_t*)faddr; 
} 
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
void STMFLASH_Write(int32_t WriteAddr,int8_t *pBuffer,int32_t NumToWrite)	
{
  int32_t endaddr=0;
  endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
/* Get FLASH_WRP_SECTORS write protection status */
HAL_FLASHEx_OBGetConfig(&OBInit);
SectorsWRPStatus = OBInit.WRPSector & FLASH_WRP_SECTORS;
/*****************************************�����д���������ֹд����***********************************************************/
if (SectorsWRPStatus == 0)    //��д����
{
/* If FLASH_WRP_SECTORS are write protected, disable the write protection */

/* Allow Access to option bytes sector */
HAL_FLASH_OB_Unlock();

/* Allow Access to Flash control registers and user Falsh */
HAL_FLASH_Unlock();

/* Disable FLASH_WRP_SECTORS write protection */
OBInit.OptionType = OPTIONBYTE_WRP;
OBInit.WRPState = WRPSTATE_DISABLE;
OBInit.Banks = FLASH_BANK_1;
OBInit.WRPSector = FLASH_WRP_SECTORS;
HAL_FLASHEx_OBProgram(&OBInit);

/* Start the Option Bytes programming process */
if (HAL_FLASH_OB_Launch() != HAL_OK)
{/* User can add here some code to deal with this error */
  while (1){}
}
/* Prevent Access to option bytes sector */
HAL_FLASH_OB_Lock();
/* Disable the Flash option control register access (recommended to protect
the option Bytes against possible unwanted operations) */
HAL_FLASH_Lock();
}
/*****************************************д�������򿪺󣬽���д�����***********************************************************/
/* Get FLASH_WRP_SECTORS write protection status */
HAL_FLASHEx_OBGetConfig(&OBInit);
SectorsWRPStatus = OBInit.WRPSector & FLASH_WRP_SECTORS;

/* Check if FLASH_WRP_SECTORS write protection is disabled */
if (SectorsWRPStatus == FLASH_WRP_SECTORS)
{
    HAL_FLASH_Unlock(); 
    
    /* Clear pending flags (if any) ���Ǳ����*/  
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
 
  FLASH_Erase_Sector(FLASH_SECTOR_3, VOLTAGE_RANGE_3 ); 
  
  while(WriteAddr<endaddr)//д����
                {
			if(HAL_FLASH_Program(TYPEPROGRAM_BYTE, WriteAddr, *pBuffer)!= HAL_OK)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=1;
			pBuffer++;
		} 

  HAL_FLASH_Lock();
}

}
//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(4λ)��
void STMFLASH_Read(int32_t ReadAddr,int8_t *pBuffer,int32_t NumToRead)   	
{
	int32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=1;//ƫ��4���ֽ�.	
	}
}




