#include "stmflash.h"

#define FLASH_WRP_SECTORS (OB_WRP_SECTOR_3 | OB_WRP_SECTOR_4) 

FLASH_OBProgramInitTypeDef OBInit;
uint32_t SectorsWRPStatus = 0xFFF;


//读取指定地址的字(32bit) 
//faddr:读地址 
//返回值:对应数据.
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
//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
//WriteAddr:起始地址(此地址必须为4的倍数!!)
//pBuffer:数据指针
//NumToWrite:字(32位)数(就是要写入的32位数据的个数.) 
void STMFLASH_Write(int32_t WriteAddr,int8_t *pBuffer,int32_t NumToWrite)	
{
  int32_t endaddr=0;
  endaddr=WriteAddr+NumToWrite*4;	//写入的结束地址
/* Get FLASH_WRP_SECTORS write protection status */
HAL_FLASHEx_OBGetConfig(&OBInit);
SectorsWRPStatus = OBInit.WRPSector & FLASH_WRP_SECTORS;
/*****************************************如果有写保护，则禁止写保护***********************************************************/
if (SectorsWRPStatus == 0)    //被写保护
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
/*****************************************写保护被打开后，进行写入操作***********************************************************/
/* Get FLASH_WRP_SECTORS write protection status */
HAL_FLASHEx_OBGetConfig(&OBInit);
SectorsWRPStatus = OBInit.WRPSector & FLASH_WRP_SECTORS;

/* Check if FLASH_WRP_SECTORS write protection is disabled */
if (SectorsWRPStatus == FLASH_WRP_SECTORS)
{
    HAL_FLASH_Unlock(); 
    
    /* Clear pending flags (if any) 不是必须的*/  
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
 
  FLASH_Erase_Sector(FLASH_SECTOR_3, VOLTAGE_RANGE_3 ); 
  
  while(WriteAddr<endaddr)//写数据
                {
			if(HAL_FLASH_Program(TYPEPROGRAM_BYTE, WriteAddr, *pBuffer)!= HAL_OK)//写入数据
			{ 
				break;	//写入异常
			}
			WriteAddr+=1;
			pBuffer++;
		} 

  HAL_FLASH_Lock();
}

}
//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(4位)数
void STMFLASH_Read(int32_t ReadAddr,int8_t *pBuffer,int32_t NumToRead)   	
{
	int32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=1;//偏移4个字节.	
	}
}




