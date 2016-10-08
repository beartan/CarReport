
#include "SysInit.h"


void RCC_Configuration(void)																											   
{  
	ErrorStatus HSEStartUpStatus; 
			
	RCC_DeInit();												
	RCC_HSEConfig(RCC_HSE_ON);									
  HSEStartUpStatus = RCC_WaitForHSEStartUp();				

	if (HSEStartUpStatus == SUCCESS)   							
	{								    
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);								
		RCC_HCLKConfig(RCC_SYSCLK_Div1);  						
		RCC_PCLK2Config(RCC_HCLK_Div1); 						
		RCC_PCLK1Config(RCC_HCLK_Div2);								
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);//PLLCLK = 8MHz * 9 = 72 MHz 	
		
		RCC_PLLCmd(ENABLE);//�ʹ��PLL���໷								
													
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}		
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//����ϵͳʱ�� == PLL ʱ��														
		while(RCC_GetSYSCLKSource() != 0x08){}		//���PLL ʱ�� �Ƿ�Ϊϵͳʱ��			
	}
}

//����STM32���ж�������
void NVIC_Configuration(void)
{
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
}

void init_sys(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	RCC_Configuration(); //ϵͳʱ�����ú���										
	NVIC_Configuration();//NVIC���ú���	 								
																
	//ʹ��APB2��������ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);	
	//�رյ��ԣ��˿�����ӳ��
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);  		
}
