
#ifndef __USART_TASK
#define __USATR_TASK

#include "Dev.h"
#include "Usart.h"

typedef __packed struct
{
	uint8_t 	magic[6];								// 魔数:"baiyue"
	uint16_t 	length;									// 整包长
	uint16_t 	crc16;									// CRC32;
	uint8_t 	version[2];							// 协议版本
	uint8_t 	cmd[8];									// 一级指令
	uint8_t 	opt[12];								// 二级指令
	uint8_t 	data[960];							// 数据区最大长度为 960 字节
} msg_packet_t;

typedef void (*msg_packet_callback)(uint8_t *data, uint16_t len);
typedef struct
{
	uint8_t cmd[8 + 1];				 //一级命令
	uint8_t opt[12 + 1];		 	 //二级命令
	msg_packet_callback p_func_callback;  //命令回调函数
}msg_packet_handle_map_t;


extern void handle_usart4_packet_task(void);
extern void status_report_task(void);

#endif

