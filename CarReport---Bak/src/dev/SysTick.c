
#include "SysTick.h" 
#include "Core.h"
#include "core_cm3.h"

soft_tmr_t g_soft_tmr[SYS_TICK_TMR_COUNT];

static uint16_t s_count = 0;
static uint32_t g_running_second = 0;

static void _init_systick(void)
{	
	//SysTick 时钟采用八分频
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	SysTick->LOAD = 9000;//SystemCoreClock = 72000000(72M),晶振为8000000(8M),即定时为 1MS
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

static void _dec_soft_tmr(soft_tmr_t *_tmr)
{
	if (_tmr->flag == 0)
	{
		if (_tmr->count > 0)
		{
			if (--_tmr->count == 0)
			{
				_tmr->flag = 1;
			}
		}
	}
}


void init_soft_tmr(void)
{
	uint8_t i;
	
	for (i = 0; i < SYS_TICK_TMR_COUNT; i++)
	{
		g_soft_tmr[i].count = 0;
		g_soft_tmr[i].flag = 0;
	}
	
	_init_systick();
	
}

void start_soft_tmr(uint8_t _id, uint32_t _period)
{
	if (_id >= SYS_TICK_TMR_COUNT)								
	{
		return;									
	}
									
	CLI();
	g_soft_tmr[_id].count = _period;
	g_soft_tmr[_id].flag = 0;  									
	SEI();
}

u8 check_soft_tmr(uint8_t _id)
{
	if (_id >= SYS_TICK_TMR_COUNT)
	{
		return 0;
	}

	if (g_soft_tmr[_id].flag == 1)
	{
		g_soft_tmr[_id].flag = 0;
		return 1;
	}
	else
	{
		return 0;
	}
}

void soft_tmr_delay_ms(uint32_t nms)
{
	if (nms <= 1)
	{
		nms = 2;
	}
	
	CLI();  									
	g_soft_tmr[SYS_TICK_TMR_ID_DELAY].count = nms;
	g_soft_tmr[SYS_TICK_TMR_ID_DELAY].flag = 0;
	SEI();

	while (1)
	{
		if (g_soft_tmr[SYS_TICK_TMR_ID_DELAY].flag == 1)
		{
			break;
		}
	}
}

uint32_t get_running_second(void)
{
	uint32_t _time = 0;
	//CLI();
	_time = g_running_second;
	//SEI();

	return _time;
}
												
void SysTick_Handler(void)
{		
		
	uint8_t i;
	
	for(i=0;i<SYS_TICK_TMR_COUNT;i++)
	{
		_dec_soft_tmr(&g_soft_tmr[i]);
	}
	
	if(++s_count >= 1000)
	{
		s_count = 0;//计时一秒
		g_running_second++;
	}
}



