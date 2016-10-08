
#ifndef __SYSTICK_H__
#define __SYSTICK_H__

#include "Dev.h"

typedef struct
{
	volatile uint32_t count;											
	volatile uint8_t flag;
}soft_tmr_t;


#define 	SYS_TICK_TMR_COUNT						4
#define		SYS_TICK_TMR_ID_DELAY		  		0											
#define		SYS_TICK_TMR_ID_STATUS				1	
#define		SYS_TICK_TMR_ID_GSM_TASK			2
#define		SYS_TICK_TMR_ID_GPS_TASK		  3




extern void init_soft_tmr(void);
extern void start_soft_tmr(u8 _id, u32 _period);
extern u8 check_soft_tmr(u8 _id);
extern void soft_tmr_delay_ms(uint32_t nms);
extern uint32_t get_running_second(void);

#endif
