
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
	uint32_t iWrite = 0 ;												  // ��д��ѭ������
  uint32_t *ReadPoint = (uint32_t *)Data;

	for (iWrite= 0; iWrite< (len/ 4); iWrite++)	  // ÿ��д4 �ֽ�
	{
		FLASH_ProgramWord(write_addr, *ReadPoint) ;  // ��ָ����ַд����
		write_addr = write_addr + 4 ;									// дָ��ƫ��
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
		//FLASH_Lock() ;											// ����FLASH 
	} 
	else if (len % 4 == 2)
	{
		*ReadPoint = (*ReadPoint) + 0xFFFF0000 ;
		FLASH_ProgramWord(write_addr, *ReadPoint) ;		
		//FLASH_Lock() ;											// ����FLASH 
	} 
	else if (len % 4 == 3)
	{
		*ReadPoint = (*ReadPoint) + 0xFF000000 ;
		FLASH_ProgramWord(write_addr, *ReadPoint) ;		
		//FLASH_Lock() ;											// ����FLASH 
	}															
}	

void work_program_from(uint32_t Addr)
{	 
	uint32_t ResetAddr ;													// ���帴λ��ַ����
	
	typedef void(*FuncPointer)(void) ;								// ����һ������ָ��
	FuncPointer	 ProgramCounterReset ;		  
					
	ResetAddr = *(uint32_t *)(Addr+4) ;									// �Ը�λ��ַ��ֵ
  ProgramCounterReset= (FuncPointer)ResetAddr ;					// ��ʼ����ջָ��    	   
	
	MSR_MSP(*(uint32_t *)Addr) ;										// ��ջ��ַд���ջָ��
	ProgramCounterReset() ;											// �Ѹ�λ��ַ����PCָ��
}

void erases_flash_page(uint32_t Addr, uint8_t PageNum)
{
	uint8_t  iPage= 0 ;													// ����ҳ�����ѭ������
	FLASH_Status  nBusy= FLASH_COMPLETE ;							// ����������״̬��־	
	for(iPage= 0; iPage< PageNum; iPage++) 
	{		  	
		nBusy= FLASH_ErasePage(Addr + 1024*iPage) ;					// ���ο�ҳ����ҳ��
		while (FLASH_COMPLETE != nBusy) ;							// �ȴ�ҳ��������
	}
}

void init_flash_memory(void)
{																
 	FLASH_Unlock() ; 			  															// ����FLASH ��д���������� 
	FLASH_ClearFlag((uint32_t)0x00000001) ;								// ���FLASH æ��־λ
	FLASH_ClearFlag((uint32_t)0x00000020) ;								// �������������־λ
	FLASH_ClearFlag((uint32_t)0x00000004) ;								// �����д�����־λ
	FLASH_ClearFlag((uint32_t)0x00000010) ;								// ���ҳ��д���������־λ
}

