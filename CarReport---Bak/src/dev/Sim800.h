
#ifndef __SIM800_H
#define __SIM800_H

#include "Dev.h"

typedef enum
{
	CMD_NONE,
	CMD_AT,//检查AT命令是否可用
	CMD_AT_ACK,
	CMD_ATE0,//设置不回显
	CMD_ATE0_ACK,
	CMD_SOCKET_CREATE,
	CMD_SOCKET_CREATE_ACK,
	CMD_SOCKET_QUERY,
	CMD_SOCKET_QUERY_ACK,
	CMD_SOCKET_SEND_START,
	CMD_SOCKET_SEND_START_ACK,
	CMD_SOCKET_SEND_DATA,
	CMD_SOCKET_SEND_STOP,
	CMD_SOCKET_SEND_STOP_ACK,
	CMD_SOCKET_CLOSE,
	CMD_SOCKET_CLOSE_ACK,
}gsm_report_cmd_t;

extern car_config_t g_car_config;

extern uint8_t cipstart_cmd[50];

extern void sim800_set_status(uint8_t status);
extern void sim800_report_task(void);
extern void init_gsm(void);
extern uint8_t gsm_status;


#endif

