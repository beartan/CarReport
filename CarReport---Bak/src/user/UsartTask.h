
#ifndef __USART_TASK
#define __USATR_TASK

#include "Dev.h"
#include "Usart.h"

typedef __packed struct
{
	uint8_t 	magic[6];								// ħ��:"baiyue"
	uint16_t 	length;									// ������
	uint16_t 	crc16;									// CRC32;
	uint8_t 	version[2];							// Э��汾
	uint8_t 	cmd[8];									// һ��ָ��
	uint8_t 	opt[12];								// ����ָ��
	uint8_t 	data[960];							// ��������󳤶�Ϊ 960 �ֽ�
} msg_packet_t;

typedef void (*msg_packet_callback)(uint8_t *data, uint16_t len);
typedef struct
{
	uint8_t cmd[8 + 1];				 //һ������
	uint8_t opt[12 + 1];		 	 //��������
	msg_packet_callback p_func_callback;  //����ص�����
}msg_packet_handle_map_t;


extern void handle_usart4_packet_task(void);
extern void status_report_task(void);

#endif

