
#include "Iwdg.h"

void init_iwdg(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  					// 使能对寄存器IWDG_PR和IWDG_RLR的写操作
	IWDG_SetPrescaler(IWDG_Prescaler_64);  							// 设置IWDG预分频值:设置IWDG预分频值为64 ;40KHz(LSI) / 64 = 625Hz */
	IWDG_SetReload(1875);  											// 设置IWDG重装载值为1875；即计时时间为3S
	IWDG_ReloadCounter();  											// 按照IWDG重装载寄存器的值重装载IWDG计数器
	IWDG_Enable();  												// 使能IWDG (the LSI oscillator will be enabled by hardware)
}
