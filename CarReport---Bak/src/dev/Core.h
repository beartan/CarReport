#ifndef __CORE_H
#define __CORE_H

#include "Dev.h"

#define CLI()      __set_PRIMASK(1) //关中断 
#define SEI()      __set_PRIMASK(0) //开中断

void INTX_DISABLE(void);
void INTX_ENABLE(void);
void MSR_MSP(uint32_t addr);

#endif
