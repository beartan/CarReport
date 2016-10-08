
#include "Sim800.h"
#include "Usart.h"
#include "string.h"
#include "SysTick.h"
#include "Core.h"
#include "Frame.h"

static gsm_report_cmd_t _gsm_report_cmd = CMD_NONE;
static uint8_t _send_opt_over[1] = {0x1A};

car_config_t g_car_config;

uint8_t cipstart_cmd[50];

uint8_t gsm_status = 0;

static void _sim800_send_cmd(uint8_t *cmd)
{
	//先清除接收缓冲区和接收长度，为本次命令腾出接收空间
	USART1_RX_STA = 0;
	memset(USART1_RX_BUF, 0, sizeof(USART1_RX_BUF));
	
	usart1_printf("%s\r\n", cmd);
	
#if DEBUG_PRINTF
	debug_printf("-------------SEND-------------\r\n");
	debug_printf("%s\r\n", cmd);
#endif
} 

static void _sim800_send_dat(uint8_t *dat, uint16_t len)
{
#if DEBUG_PRINTF	
	uint16_t index = 0;
#endif

	//先清除接收缓冲区和接收长度，为本次命令腾出接收空间
	USART1_RX_STA = 0;
	memset(USART1_RX_BUF, 0, sizeof(USART1_RX_BUF));
	
	usart1_send_data(dat, len);

#if DEBUG_PRINTF
	debug_printf("-------------SEND-------------\r\n");
	for(index = 0; index < len; index++)
	{
		debug_printf("%02X ", dat[index]);
	}
	debug_printf("\r\n");
#endif
} 


uint8_t* _sim800_check_ack(uint8_t *ack)
{
	char *strx = 0;
	
	USART1_RX_STA |= 1<<15;
	USART1_RX_BUF[USART1_RX_STA & 0x7FFF] = 0;//添加结束符
						
	strx = strstr((const char*)USART1_RX_BUF, (const char*)ack);

#if DEBUG_PRINTF
	debug_printf("=============RECV=============\r\n");
	debug_printf("%s\r\n", USART1_RX_BUF);
#endif

	return (uint8_t*)strx;
}

static void _sim800_change_status(gsm_report_cmd_t status, uint16_t microsecond)
{
	//CLI();
	_gsm_report_cmd = status;
	//SEI();
	start_soft_tmr( SYS_TICK_TMR_ID_GSM_TASK, microsecond );
}

void sim800_report_task(void)
{
	if( !check_soft_tmr(SYS_TICK_TMR_ID_GSM_TASK) )
	{
		return;
	}

	switch(_gsm_report_cmd)
	{
		case CMD_NONE:
			GPIO_ResetBits(GPIOC, GPIO_Pin_5);
			_sim800_change_status(CMD_AT, 1000);
			break;
			
		case CMD_AT:
			_sim800_send_cmd("AT");
			_sim800_change_status(CMD_AT_ACK, 1000);
			break;
			
		case CMD_AT_ACK:
			if(_sim800_check_ack("OK"))
			{
				_sim800_change_status(CMD_ATE0, 1000);
			}
			else
			{
				_sim800_change_status(CMD_AT, 1000);
			}
			break;

		case CMD_ATE0:
			_sim800_send_cmd("ATE0");
			_sim800_change_status(CMD_ATE0_ACK, 100);
			break;

		case CMD_ATE0_ACK:
			if(_sim800_check_ack("OK"))
			{
				_sim800_change_status(CMD_SOCKET_CREATE, 1000);
			}
			else
			{
				_sim800_change_status(CMD_AT, 100);
			}
			break;

		case CMD_SOCKET_CREATE:
			//_sim800_send_cmd("AT+CIPSTART=\"TCP\",\"139.196.164.147\",\"2347\"");
			_sim800_send_cmd(cipstart_cmd);
			_sim800_change_status(CMD_SOCKET_CREATE_ACK, 4000);
			break;

		case CMD_SOCKET_CREATE_ACK:
			if(_sim800_check_ack("CONNECT OK"))
			{
				_sim800_change_status(CMD_SOCKET_SEND_START, 1000);
			}
			else if(_sim800_check_ack("ALREADY CONNECT"))
			{
				_sim800_change_status(CMD_SOCKET_SEND_START, 1000);
			}
			else
			{
				_sim800_change_status(CMD_SOCKET_QUERY, 1000);
			}
			break;

		case CMD_SOCKET_QUERY:
			_sim800_send_cmd("AT+CIPSTATUS");
			_sim800_change_status(CMD_SOCKET_QUERY_ACK, 1000);
			break;

		case CMD_SOCKET_QUERY_ACK:
			if(_sim800_check_ack("CONNECT OK"))
			{
				_sim800_change_status(CMD_SOCKET_SEND_START, 1000);
			}
			else
			{
				_sim800_change_status(CMD_SOCKET_CREATE, 1000);
			}
			break;

		case CMD_SOCKET_SEND_START:
			gsm_status = 1;
			_sim800_send_cmd("AT+CIPSEND");
			_sim800_change_status(CMD_SOCKET_SEND_START_ACK, 200);
			break;

		case CMD_SOCKET_SEND_START_ACK:
			if( _sim800_check_ack(">"))
			{
				_sim800_change_status(CMD_SOCKET_SEND_DATA, 200);
			}
			else
			{
				_sim800_change_status(CMD_SOCKET_CLOSE, 500);
			}
			break;

		case CMD_SOCKET_SEND_DATA:
			set_frame_data();
			_sim800_send_dat(get_frame_data(), get_frame_size());
			_sim800_change_status(CMD_SOCKET_SEND_STOP, 2300);
			break;

		case CMD_SOCKET_SEND_STOP:
			_sim800_send_dat(_send_opt_over, sizeof(_send_opt_over));
			_sim800_change_status(CMD_SOCKET_SEND_STOP_ACK, 2200);
			break;

		case CMD_SOCKET_SEND_STOP_ACK:
			if(_sim800_check_ack("SEND OK"))
			{
				_sim800_change_status(CMD_SOCKET_SEND_START, 500);
			}
			else
			{
				_sim800_change_status(CMD_SOCKET_QUERY, 500);
			}
			break;

		case CMD_SOCKET_CLOSE:
			_sim800_send_cmd("AT+CIPCLOSE");
			_sim800_change_status(CMD_SOCKET_CLOSE_ACK, 200);
			break;

		case CMD_SOCKET_CLOSE_ACK:
			if(_sim800_check_ack("CLOSE OK"))
			{
				_sim800_change_status(CMD_SOCKET_CREATE, 2000);
			}
			else
			{
				_sim800_change_status(CMD_AT, 1000);
			}
			break;

		default:
			_sim800_change_status(CMD_AT, 1000);
			break;
	}
}


void init_gsm(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 					// 蜂鸣器
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_5);
}

