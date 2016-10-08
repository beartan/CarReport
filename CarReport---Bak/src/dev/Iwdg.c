
#include "Iwdg.h"

void init_iwdg(void)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  					// ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
	IWDG_SetPrescaler(IWDG_Prescaler_64);  							// ����IWDGԤ��Ƶֵ:����IWDGԤ��ƵֵΪ64 ;40KHz(LSI) / 64 = 625Hz */
	IWDG_SetReload(1875);  											// ����IWDG��װ��ֵΪ1875������ʱʱ��Ϊ3S
	IWDG_ReloadCounter();  											// ����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
	IWDG_Enable();  												// ʹ��IWDG (the LSI oscillator will be enabled by hardware)
}
