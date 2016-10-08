#ifndef __FRAME_H
#define __FRAME_H

#include "Dev.h"

enum
{
	HY_PROTOCOL = 0,
	AX_PROTOCOL,
	XX_PROTOCOL,
	CGQC_PROTOCOL,
	DYKJ_PROTOCOL,
	JSYT_PROTOCOL,
	VICT_PROTOCOL,
	YDZY_PROTOCOL
};

extern void set_frame_data(void);
extern uint8_t *get_frame_data(void);
extern uint16_t get_frame_size(void);

#endif
