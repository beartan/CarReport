
#ifndef __USART_H
#define __USART_H

#include "Dev.h"								 // ����������ͷ�ļ�

#define USART1_REMAP_PB06_PB07_FOR_DEBUG  0//1.�� USART1 ӳ�䵽 PB06 PB07��  0.������������GSM
#define USART3_REMAP_PC10_PC11_FOR_DEBUG  0//1.�� USART3 ӳ�䵽 PC10 PC11��  0.������������GPS
#define DEBUG_PRINTF											1

#define USART_BASE_LEN				600
#define USART3_GPS_RECV_BUF   (2*USART_BASE_LEN)

//��������֡����
#define   FRAME_MAG_0_VAL     0x3A
#define   FRAME_MAG_1_VAL     0xA3

#define   FRAME_MAG_POS  	   0
#define   FRAME_MAG_LEN  	   6
#define   FRAME_LEN_0_POS    6
#define   FRAME_LEN_1_POS    7
#define   FRAME_LEN_LEN      2
#define   FRAME_VER_0_POS    8
#define   FRAME_VER_1_POS    9
#define   FRAME_VER_LEN      2
#define   FRAME_CRC_0_POS   10
#define   FRAME_CRC_1_POS  	11	
#define   FRAME_CRC_LEN  		 2
#define   FRAME_CMD_POS  		12
#define   FRAME_CMD_LEN  		 8
#define   FRAME_OPT_POS  	  20
#define   FRAME_OPT_LEN  	  12
#define   FRAME_DATA_POS    32
#define   FRAME_HEAD_LEN  	32


typedef struct
{
	uint8_t  	*rx_buf;											// ���ջ����� 
	uint16_t 	rx_buf_size;									// ���ջ�������С 
	uint16_t 	rx_write;											// ���ջ�����дָ��
	uint16_t 	rx_read;											// ���ջ�������ָ��
	
	void (*rx_byte_finish)(void);						// �յ����ݵĻص�����ָ��
}usart_queue_t;

extern uint8_t debug_state;

extern void init_usart(void);
extern void usart1_printf(char* fmt, ...);
extern void usart1_send_data(uint8_t *buf, uint16_t len);
extern void usart3_printf(char* fmt, ...);
extern void debug_printf(char* fmt, ...);

extern uint8_t usart4_recv_packet(uint8_t *buf, uint16_t *len);
extern void usart4_send_packet(uint8_t *p_buf, uint16_t len);

extern uint16_t USART1_RX_STA;
extern uint8_t USART1_RX_BUF[USART_BASE_LEN]; //���ջ���,���USART_BASE_LEN���ֽ�.
extern uint16_t USART3_RX_STA;
extern uint8_t USART3_RX_BUF[USART3_GPS_RECV_BUF]; //���ջ���,���USART_BASE_LEN���ֽ�.


	  	
#endif

