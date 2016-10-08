#ifndef __CORE_H
#define __CORE_H

#include "Dev.h"

#define CLI()      __set_PRIMASK(1) //���ж� 
#define SEI()      __set_PRIMASK(0) //���ж�

void INTX_DISABLE(void);
void INTX_ENABLE(void);
void MSR_MSP(uint32_t addr);

#endif
