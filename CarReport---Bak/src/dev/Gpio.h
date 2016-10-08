
#ifndef __GPIO_H
#define __GPIO_H

#include "Dev.h"

extern void init_led(void);

#define SET_POWER()           GPIO_SetBits(GPIOB, GPIO_Pin_6)
#define CLR_POWER()           GPIO_ResetBits(GPIOB, GPIO_Pin_6)
#define SET_GSM_SEND_BYTE()   GPIO_SetBits(GPIOB, GPIO_Pin_7)
#define CLR_GSM_SEND_BYTE()   GPIO_ResetBits(GPIOB, GPIO_Pin_7)
#define SET_GSM_SEND_FRAME()  GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define CLR_GSM_SEND_FRAME()  GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define SET_GPS_RECV_DATA()   GPIO_SetBits(GPIOB, GPIO_Pin_9)
#define CLR_GPS_RECV_DATA()   GPIO_ResetBits(GPIOB, GPIO_Pin_9)


#endif
