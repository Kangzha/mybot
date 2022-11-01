#include "stm32f10x_flash.h"

void FLASH_WriteByte(uint32_t addr , uint8_t *p , uint16_t Byte_Num)
{
	uint32_t HalfWord;
  Byte_Num = Byte_Num/2;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(addr);
  while(Byte_Num--)
	{
		HalfWord=*(p++);
		HalfWord|=*(p++)<<8;
		FLASH_ProgramHalfWord(addr,HalfWord);
		addr +=2;
	}
	FLASH_Lock();
}

void FLASH_ReadByte(uint32_t addr, uint8_t *p, uint16_t Byte_Num)
{
	while(Byte_Num--)
	{
		*(p++)=*((uint8_t*)addr++);
	}

}