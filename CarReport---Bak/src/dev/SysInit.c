
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
		
		RCC_PLLCmd(ENABLE);//剖鼓躊LL锁相环								
													
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}		
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//配置系统时钟 == PLL 时钟														
		while(RCC_GetSYSCLKSource() != 0x08){}		//检查PLL 时钟 是否为系统时钟			
	}
}

//配置STM32的中断向量表
void NVIC_Configuration(void)
{
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
}

void init_sys(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	RCC_Configuration(); //系统时钟配置函数										
	NVIC_Configuration();//NVIC配置函数	 								
																
	//使能APB2总线外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);	
	//关闭调试，端口重新映射
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);  		
}
