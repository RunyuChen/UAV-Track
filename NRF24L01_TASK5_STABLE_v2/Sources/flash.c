#include "flash.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

uint32_t Address = 0x00;

static uint32_t GetSector(uint32_t Address);

void MEM_If_Init_FS(void)
{
    HAL_FLASH_Unlock(); 
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

uint16_t MEM_If_Erase_FS(uint32_t start_Add,uint32_t end_Add)
{
    uint32_t UserStartSector;
    uint32_t SectorError;
    FLASH_EraseInitTypeDef pEraseInit;
 
    /* Unlock the Flash to enable the flash control register access *************/
    MEM_If_Init_FS();
 
    /* Get the sector where start the user flash area */
    UserStartSector = GetSector(start_Add);
 
    pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    pEraseInit.Sector = UserStartSector;
    pEraseInit.NbSectors = GetSector(end_Add)-UserStartSector+1 ;
    pEraseInit.VoltageRange = VOLTAGE_RANGE_3;
 
    if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
    {
        /* Error occurred while page erase */
        return (1);
    }
    return 0;
}

uint8_t writedata_to_flash(int16_t *data,uint32_t len,uint32_t address)
{
   static int16_t tmp;
   uint32_t i=0;

   for(i=0;i<len;i++)
   { tmp=*((__IO uint16_t *)address+i);
     if(data[i]!=tmp)
       break;
   }
   if (i==len) //if待定入的值与flash中的值完全一样
     return 1;
   //MEM_If_Erase_FS(FLASH_Start_Addr,FLASH_End_Addr);
   //FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
   for(i=0;i<len;i++)
   {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)address, *data) == HAL_OK)
      {
        data++;
        address = address + 2;
      }
     else
      {
        return 0;
      }
   }
   HAL_FLASH_Lock();
   return 0;
 }

void flash_read(uint32_t *data,uint8_t len)
{
  Address = FLASH_Start_Addr;
  
  while (len--)
  {
    *data = *(__IO uint32_t *)Address;
    data++;
    Address = Address + 4;
  }
}


/**
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;  
  }
  else
  {
    sector = FLASH_SECTOR_5;  
  }

  return sector;
}

/*****END OF FILE****/
