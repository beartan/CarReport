
#include "UsartTask.h"
#include "Usart.h"
#include "Tool.h"
#include "SysTick.h"
#include "Core.h"
#include "Sim800.h"
#include "Flash.h"
#include "Gstar.h"
#include "Usart.h"

//软硬件版本信息，方便后期维护和升级
const uint8_t c_soft_ver = 100;//表示软件版本号码为 1.00
const uint8_t c_hard_ver = 100;//表示硬件版本号码为 1.00

const uint8_t packet_vision[2] = {0x00, 0x01};
static uint8_t recv_packet[USART_BASE_LEN] = {0};
static uint8_t send_packet[USART_BASE_LEN] = {0};

void _packet_send(uint8_t *cmd, uint8_t *opt, uint8_t *dat_buf, uint8_t dat_len)
{
	msg_packet_t *p_packet = (msg_packet_t *)(send_packet);

	memset(send_packet, 0, sizeof(send_packet));

	strncpy((char *)p_packet->magic, "baiyue", sizeof(p_packet->magic));
	
	p_packet->length = FRAME_HEAD_LEN + dat_len;

	p_packet->version[0] = packet_vision[0];
	p_packet->version[1] = packet_vision[1];

	strncpy((char *)p_packet->cmd, (char *)cmd, sizeof(p_packet->cmd));
	strncpy((char *)p_packet->opt, (char *)opt, sizeof(p_packet->opt));

	memcpy((char *)p_packet->data, (char *)dat_buf, dat_len);

	p_packet->crc16 = cal_crc16((uint8_t *)p_packet + 10, p_packet->length - 10);

	usart4_send_packet((uint8_t *)p_packet, p_packet->length);
}


static uint8_t _packet_parse(uint16_t *data_len)
{
	msg_packet_t *p_packet = (msg_packet_t *)(recv_packet);
	
	//检查帧头魔数是否正确
	if(strncmp((const char *)p_packet->magic, "baiyue", 6) != 0)
	{
		return RET_FALSE;
	}

	//检查帧头协议版本是否正确
	if(p_packet->version[0] != packet_vision[0] || p_packet->version[1] != packet_vision[1])
	{
		return RET_FALSE;
	}

	//--- 由于STM32为小端格式，协议也为小端格式，因此可以直接指针读取，
	//--- 如有移植代码请注意!!!!
	if(p_packet->length > USART_BASE_LEN)
	{
		return RET_FALSE;
	}

	if(cal_crc16((uint8_t *)p_packet + 10, p_packet->length - 10) != p_packet->crc16)
	{
		return RET_FALSE;
	}

	*data_len = p_packet->length - FRAME_HEAD_LEN;

	return RET_TRUE;
}

//协议格式为:"{}"
static void _callback_handler_cent_resetfac(uint8_t *data, uint16_t len)
{ 
	init_flash_memory();
	erases_flash_page(FLASH_CONFIG_BASE_ADDR, 1);
	_packet_send("CENT", "resetfac", "{}", 2);
}

//协议格式为:"{ipaddr1=192;ipaddr1=168;ipaddr1=1;ipaddr1=112;ipport=8080;}"
static void _callback_handler_cent_ipconfig(uint8_t *data, uint16_t len)
{ 
	uint8_t send_data[100] = {0};
	uint8_t ipaddr1 = 0, ipaddr2 = 0, ipaddr3 = 0, ipaddr4 = 0;
	uint16_t ipport = 0;

	ipaddr1	= (uint8_t)tool_get_key_int((char *)data, "ipaddr1");
	ipaddr2	= (uint8_t)tool_get_key_int((char *)data, "ipaddr2");
	ipaddr3	= (uint8_t)tool_get_key_int((char *)data, "ipaddr3");
	ipaddr4	= (uint8_t)tool_get_key_int((char *)data, "ipaddr4");
	ipport	= (uint16_t)tool_get_key_int((char *)data, "ipport");

	if(ipaddr1 != 0 && ipaddr2 != 0 && ipaddr3 != 0 && ipaddr4 != 0 && ipport != 0) 
	{
		memset(g_car_config.ipaddr_str, 0 ,sizeof(g_car_config.ipaddr_str));
		memset(g_car_config.ipport_str, 0 ,sizeof(g_car_config.ipport_str));
		sprintf((char *)g_car_config.ipaddr_str, "%d.%d.%d.%d", ipaddr1,ipaddr2,ipaddr3,ipaddr4);
		sprintf((char *)g_car_config.ipport_str, "%d", ipport);
		
		init_flash_memory();
		erases_flash_page(FLASH_CONFIG_BASE_ADDR, 1);
		memcpy(g_car_config.indentify, (void *)"evideo", 6);
		g_car_config.crc16 = cal_crc16((uint8_t *)&g_car_config, sizeof(car_config_t) - 2);

		write_data_to_rom(FLASH_CONFIG_BASE_ADDR, (uint8_t *)&g_car_config, sizeof(car_config_t) );

		memset((char *)send_data, 0, sizeof(send_data));
		sprintf((char *)send_data, "{ipaddr1=%d;ipaddr2=%d;ipaddr3=%d;ipaddr4=%d;ipport=%d;ret=1;}", ipaddr1,ipaddr2,ipaddr3,ipaddr4,ipport);
		_packet_send("CONFIG", "ipconfig", (uint8_t *)send_data, strlen((char *)send_data));
	}
}

//协议格式为:"{}"
static void _callback_handler_cent_protocol(uint8_t *data, uint16_t len)
{ 
	uint8_t send_data[100] = {0};
	uint8_t protocol;

	protocol	= (uint8_t)tool_get_key_int((char *)data, "protocol");

	g_car_config.protocol_type = protocol;
	
	init_flash_memory();
	erases_flash_page(FLASH_CONFIG_BASE_ADDR, 1);
	memcpy(g_car_config.indentify, (void *)"evideo", 6);
	g_car_config.crc16 = cal_crc16((uint8_t *)&g_car_config, sizeof(car_config_t) - 2);

	write_data_to_rom(FLASH_CONFIG_BASE_ADDR, (uint8_t *)&g_car_config, sizeof(car_config_t) );

	memset((char *)send_data, 0, sizeof(send_data));
	sprintf((char *)send_data, "{protocol=%d;ret=1;}", protocol);
	_packet_send("CONFIG", "protocol", (uint8_t *)send_data, strlen((char *)send_data));
}


static void _callback_handler_cent_carvin(uint8_t *data, uint16_t len)
{ 
	uint8_t send_data[100] = {0};
	char carvin_str[17];
	int contentLen;
	char *content = NULL;

	
	content = tool_get_key_str( (char *)data, "carvinstr", &contentLen);			
	if( content && contentLen == 17)
	{
		memset(carvin_str, 0, sizeof(carvin_str));
		strncpy(carvin_str, content, contentLen);

		g_car_config.car_vin[0x00] = carvin_str[0x00];
		g_car_config.car_vin[0x01] = carvin_str[0x01];
		g_car_config.car_vin[0x02] = carvin_str[0x02];
		g_car_config.car_vin[0x03] = carvin_str[0x03];
		g_car_config.car_vin[0x04] = carvin_str[0x04];
		g_car_config.car_vin[0x05] = carvin_str[0x05];
		g_car_config.car_vin[0x06] = carvin_str[0x06];
		g_car_config.car_vin[0x07] = carvin_str[0x07];
		g_car_config.car_vin[0x08] = carvin_str[0x08];
		g_car_config.car_vin[0x09] = carvin_str[0x09];
		g_car_config.car_vin[0x0A] = carvin_str[0x0A];
		g_car_config.car_vin[0x0B] = carvin_str[0x0B];
		g_car_config.car_vin[0x0C] = carvin_str[0x0C];
		g_car_config.car_vin[0x0D] = carvin_str[0x0D];
		g_car_config.car_vin[0x0E] = carvin_str[0x0E];
		g_car_config.car_vin[0x0F] = carvin_str[0x0F];
		g_car_config.car_vin[0x10] = carvin_str[0x10];
		
		init_flash_memory();
		erases_flash_page(FLASH_CONFIG_BASE_ADDR, 1);
		memcpy(g_car_config.indentify, (void *)"evideo", 6);
		g_car_config.crc16 = cal_crc16((uint8_t *)&g_car_config, sizeof(car_config_t) - 2);

		write_data_to_rom(FLASH_CONFIG_BASE_ADDR, (uint8_t *)&g_car_config, sizeof(car_config_t) );

		memset((char *)send_data, 0, sizeof(send_data));
		sprintf((char *)send_data, "{ret=1;}");
		_packet_send("CONFIG", "carvin", (uint8_t *)send_data, strlen((char *)send_data));
	}
}


static void _callback_handler_cent_debug(uint8_t *data, uint16_t len)
{ 
	uint8_t send_data[100] = {0};
	uint8_t debug;

	debug	= (uint8_t)tool_get_key_int((char *)data, "debug");

	debug_state = debug;

	memset((char *)send_data, 0, sizeof(send_data));
	sprintf((char *)send_data, "{debug=%d;ret=1;}", debug);
	_packet_send("CONFIG", "debug", (uint8_t *)send_data, strlen((char *)send_data));
}


static const msg_packet_handle_map_t g_msg_packet_handle_map[] = 
{
	{"CENT", 				"resetfac",    	 _callback_handler_cent_resetfac},  //恢复出厂
	{"CONFIG", 			"ipconfig",    	 _callback_handler_cent_ipconfig},  //地址设置
	{"CONFIG", 			"protocol",    	 _callback_handler_cent_protocol},  //协议设置
	{"CONFIG", 			"carvin",    	 	 _callback_handler_cent_carvin},  //协议设置
	{"CONFIG", 			"debug",    	 	 _callback_handler_cent_debug},  //协议设置
};

void handle_usart4_packet_task(void)
{

	uint8_t i = 0;
	uint16_t packet_len = 0;
	uint16_t data_len = 0;
	uint8_t *p_data;

	msg_packet_t *p_packet = (msg_packet_t *)(recv_packet);
	
	memset(recv_packet, 0, sizeof(recv_packet));
	if(usart4_recv_packet(recv_packet, &packet_len) != RET_TRUE)
	{
		return;
	}

	if(_packet_parse(&data_len) != RET_TRUE)
	{
		return;
	}
	
	p_data = p_packet->data;

	for (i = 0; i < (int)(sizeof(g_msg_packet_handle_map) / sizeof(msg_packet_handle_map_t)); i++ )
	{
		//一级指令检查 
		if (0 == strncmp((const char *)p_packet->cmd, (const char *)g_msg_packet_handle_map[i].cmd, 8))
		{
			//二级指令检查
			if(0 == strncmp((const char*)p_packet->opt, (const char *)g_msg_packet_handle_map[i].opt, 12))
			{
				//回调函数检查
				if(g_msg_packet_handle_map[i].p_func_callback)
				{
					(*g_msg_packet_handle_map[i].p_func_callback)(p_data, data_len);
				}
			}
		}
	}
}



void status_report_task()
{
	if( !check_soft_tmr(SYS_TICK_TMR_ID_STATUS) )
	{
		return;
	}

	if(gsm_status == 1 )
	{
		_packet_send("CENT", "gsmstatus", "{}", 2);
	}

	soft_tmr_delay_ms(100);

	if(g_status == 0 )
	{
		_packet_send("CENT", "gpsstatus", "{}", 2);
	}

	start_soft_tmr( SYS_TICK_TMR_ID_STATUS, 3000 );
}

