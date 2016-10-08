
#include "Flash.h"
#include "Core.h"

void read_data_from_flash(uint32_t flashAddr, uint8_t *pbuf, uint16_t len)
{
	u16 i;
	for(i=0;i<len;i++)
	{
		pbuf[i] = *(vu8 *)(flashAddr+i);
	}
}


void write_data_to_rom(uint32_t flash_addr, uint8_t *Data, uint16_t len)
{
	uint32_t write_addr = flash_addr ;									
	uint32_t iWrite = 0 ;												  // 读写的循环变量
  uint32_t *ReadPoint = (uint32_t *)Data;

	for (iWrite= 0; iWrite< (len/ 4); iWrite++)	  // 每次写4 字节
	{
		FLASH_ProgramWord(write_addr, *ReadPoint) ;  // 在指定地址写数据
		write_addr = write_addr + 4 ;									// 写指针偏移
		ReadPoint++;
	}
	
	if (len % 4 == 0)
	{
		//FLASH_Lock() ;
	} 
	else if (len % 4 == 1)
	{
		*ReadPoint = (*ReadPoint) + 0xFFFFFF00 ;
		FLASH_ProgramWord(write_addr, *ReadPoint) ;		
		//FLASH_Lock() ;											// 锁定FLASH 
	} 
	else if (len % 4 == 2)
	{
		*ReadPoint = (*ReadPoint) + 0xFFFF0000 ;
		FLASH_ProgramWord(write_addr, *ReadPoint) ;		
		//FLASH_Lock() ;											// 锁定FLASH 
	} 
	else if (len % 4 == 3)
	{
		*ReadPoint = (*ReadPoint) + 0xFF000000 ;
		FLASH_ProgramWord(write_addr, *ReadPoint) ;		
		//FLASH_Lock() ;											// 锁定FLASH 
	}															
}	

void work_program_from(uint32_t Addr)
{	 
	uint32_t ResetAddr ;													// 定义复位地址变量
	
	typedef void(*FuncPointer)(void) ;								// 定义一个函数指针
	FuncPointer	 ProgramCounterReset ;		  
					
	ResetAddr = *(uint32_t *)(Addr+4) ;									// 对复位地址赋值
  ProgramCounterReset= (FuncPointer)ResetAddr ;					// 初始化堆栈指针    	   
	
	MSR_MSP(*(uint32_t *)Addr) ;										// 堆栈地址写入堆栈指针
	ProgramCounterReset() ;											// 把复位地址赋给PC指针
}

void erases_flash_page(uint32_t Addr, uint8_t PageNum)
{
	uint8_t  iPage= 0 ;													// 定义页面擦除循环变量
	FLASH_Status  nBusy= FLASH_COMPLETE ;							// 定义操作完成状态标志	
	for(iPage= 0; iPage< PageNum; iPage++) 
	{		  	
		nBusy= FLASH_ErasePage(Addr + 1024*iPage) ;					// 依次跨页擦除页面
		while (FLASH_COMPLETE != nBusy) ;							// 等待页面擦除完成
	}
}

void init_flash_memory(void)
{																
 	FLASH_Unlock() ; 			  															// 解锁FLASH 编写擦除控制器 
	FLASH_ClearFlag((uint32_t)0x00000001) ;								// 清除FLASH 忙标志位
	FLASH_ClearFlag((uint32_t)0x00000020) ;								// 清除操作结束标志位
	FLASH_ClearFlag((uint32_t)0x00000004) ;								// 清除编写错误标志位
	FLASH_ClearFlag((uint32_t)0x00000010) ;								// 清除页面写保护错误标志位
}

