
/**************************************************************
*	注意事项:
* 1.驼峰结构的代码为 ST 官方库函数
* 2.Linux小写加下划线结构代码为 百越创科工作室 定义函数
**************************************************************/

#include "SysInit.h"
#include "Iwdg.h"
#include "Usart.h"
#include "SysTick.h"
#include "Gpio.h"
#include "Sim800.h"
#include "Can.h"
#include "Gstar.h"
#include "UsartTask.h"
#include "Flash.h"
#include "Tool.h"
#include "Frame.h"

void init_config(void) 
{
	uint8_t i = 0;
	car_config_t tmp_config;
	
	memset(&g_car_config, 0, sizeof(car_config_t));
	strlcat((char *)g_car_config.ipaddr_str, "139.196.164.147", sizeof(g_car_config.ipaddr_str));
	strlcat((char *)g_car_config.ipport_str, "2347", sizeof(g_car_config.ipport_str));
	g_car_config.protocol_type = AX_PROTOCOL;

#if 0	
	// 100000 000000 00000
	g_car_config.car_vin[0x00] = '1';
	g_car_config.car_vin[0x01] = '0';
	g_car_config.car_vin[0x02] = '0';
	g_car_config.car_vin[0x03] = '0';
	g_car_config.car_vin[0x04] = '0';
	g_car_config.car_vin[0x05] = '0';
	g_car_config.car_vin[0x06] = '0';
	g_car_config.car_vin[0x07] = '0';
	g_car_config.car_vin[0x08] = '0';
	g_car_config.car_vin[0x09] = '0';
	g_car_config.car_vin[0x0A] = '0';
	g_car_config.car_vin[0x0B] = '0';
	g_car_config.car_vin[0x0C] = '0';
	g_car_config.car_vin[0x0D] = '0';
	g_car_config.car_vin[0x0E] = '0';
	g_car_config.car_vin[0x0F] = '0';
	g_car_config.car_vin[0x10] = '0';
#else
	// LA9BX5B43F1JFZ427
	g_car_config.car_vin[0x00] = 'L';
	g_car_config.car_vin[0x01] = 'A';
	g_car_config.car_vin[0x02] = '9';
	g_car_config.car_vin[0x03] = 'B';
	g_car_config.car_vin[0x04] = 'X';
	g_car_config.car_vin[0x05] = '5';
	g_car_config.car_vin[0x06] = 'B';
	g_car_config.car_vin[0x07] = '4';
	g_car_config.car_vin[0x08] = '3';
	g_car_config.car_vin[0x09] = 'F';
	g_car_config.car_vin[0x0A] = '1';
	g_car_config.car_vin[0x0B] = 'J';
	g_car_config.car_vin[0x0C] = 'F';
	g_car_config.car_vin[0x0D] = 'Z';
	g_car_config.car_vin[0x0E] = '4';
	g_car_config.car_vin[0x0F] = '2';
	g_car_config.car_vin[0x10] = '7';
#endif

	memset(&tmp_config, 0, sizeof(tmp_config));
	read_data_from_flash(FLASH_CONFIG_BASE_ADDR, (uint8_t *)&tmp_config, sizeof(tmp_config));
	if(memcmp(tmp_config.indentify, "evideo", 6) == 0 && tmp_config.crc16 == cal_crc16((uint8_t *)&tmp_config, sizeof(car_config_t)-2))
	{
		memcpy(g_car_config.ipaddr_str, tmp_config.ipaddr_str, sizeof(g_car_config.ipaddr_str));
		memcpy(g_car_config.ipport_str, tmp_config.ipport_str, sizeof(g_car_config.ipport_str));
		g_car_config.protocol_type = tmp_config.protocol_type;
		memcpy(g_car_config.car_vin, tmp_config.car_vin, sizeof(g_car_config.car_vin));
	  debug_printf("read config success\r\n");
	}
	else
	{
	  debug_printf("read config failure\r\n");
	}
	debug_printf("------------------------------------------------\r\n");
  debug_printf("g_car_config.ipaddr_str=%s\r\n", g_car_config.ipaddr_str);
	debug_printf("g_car_config.ipport_str=%s\r\n", g_car_config.ipport_str);
	debug_printf("g_car_config.protocol_type=%d\r\n", g_car_config.protocol_type);
	debug_printf("VIN:");
	for(i = 0; i < 17; i++)
	{
		debug_printf("%c", g_car_config.car_vin[i]);
	}
	debug_printf("\r\n------------------------------------------------\r\n");

	memset(cipstart_cmd, 0, sizeof(cipstart_cmd));
	sprintf((char*)cipstart_cmd, "AT+CIPSTART=\"TCP\",\"%s\",\"%s\"", g_car_config.ipaddr_str, g_car_config.ipport_str);
}

int main()
{
	init_soft_tmr();		// 初始化系统滴答时钟和软件定时器
	init_sys();
	init_iwdg();			// 初始化看门狗
	init_usart();			// 初始化串口	
	init_led();
	init_gsm();
	init_can();
	init_config();
	
	start_soft_tmr( SYS_TICK_TMR_ID_STATUS, 1000 );
	start_soft_tmr( SYS_TICK_TMR_ID_GSM_TASK, 6000 );//GSM
	start_soft_tmr( SYS_TICK_TMR_ID_GPS_TASK, 5000 );//GPS
	
	for(;;)
	{
		IWDG_ReloadCounter();  					// 清狗;按照IWDG重装载寄存器的值重装载IWDG计数器
		handle_usart4_packet_task();
		sim800_report_task();
		gstar_report_task();
		status_report_task();
	}
}
