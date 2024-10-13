#ifndef _FLASH_H
#define _FLASH_H

#define FLASH_Page_Size    (0x00000400) //FLASH页的大小
#define FLASH_Start_Addr   (0x08010000) //要编写的起始地址
#define FLASH_End_Addr     (0x0801FFFF) //要编写的结束地址
#define DATA_32            (0x1234567) //要编写的数据

//sector起始地址
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbyte */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbyte */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbyte */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbyte */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbyte */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbyte */

#include "main.h"
uint8_t writedata_to_flash(int16_t *data,uint32_t len,uint32_t address);
void flash_read(uint32_t *data,uint8_t len);
#endif 

