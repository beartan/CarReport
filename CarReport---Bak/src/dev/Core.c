
#include "Core.h"

__asm void INTX_DISABLE(void)
{
	CPSID I;		  
}
__asm void INTX_ENABLE(void)
{
	CPSIE I;		  
}

__asm void MSR_MSP(uint32_t addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}
