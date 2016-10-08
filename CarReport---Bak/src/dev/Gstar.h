
#ifndef __GSTAR_H
#define __GSTAR_H

#include "Dev.h"

extern uint8_t g_status;
extern uint32_t g_longitude;
extern uint32_t g_latitude;
extern uint32_t g_speed;
extern uint32_t g_course;
extern uint32_t g_ddmmyy;
extern uint32_t g_hhmmss;


extern void gstar_report_task(void);

#endif

