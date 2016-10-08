#ifndef __DEV_H__
#define __DEV_H__

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f10x_conf.h"

#include "SysInit.h"  

#define  null  ((void *)0)
#define	 NULL							0


#define  RET_FALSE		0
#define  RET_TRUE     1

typedef struct
{
	uint8_t indentify[6];
	uint8_t ipaddr_str[16];
	uint8_t ipport_str[6];
	uint8_t protocol_type;
	uint8_t car_vin[17];
	uint16_t crc16;
}car_config_t;


#endif
