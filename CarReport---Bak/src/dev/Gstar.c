
#include "Gstar.h"
#include "Usart.h"
#include "string.h"
#include "SysTick.h"
#include "Core.h"
#include "Tool.h"

static uint8_t _search_buf[100];
uint8_t g_status = 1;
uint32_t g_longitude = 0;
uint32_t g_latitude = 0;
uint32_t g_speed = 0;
uint32_t g_course = 0;
uint32_t g_ddmmyy = 0;
uint32_t g_hhmmss = 0;



void gstar_report_task(void)
{
	uint8_t *buf = NULL;
	uint8_t val_status = 0;
	uint32_t val_longitude = 0;
	uint32_t val_latitude = 0;
	uint16_t val_speed = 0;
	uint16_t val_dir = 0;
	uint32_t val_ddmmyy;
	uint32_t val_hhmmss;
	uint8_t str_status[2];
	uint8_t str_longitude[15];
	uint8_t str_latitude[15];
	uint8_t str_speed[10];
	uint8_t str_dir[10];
	uint8_t str_ddmmyy[10];
	uint8_t str_hhmmss[10];
	
	if( !check_soft_tmr(SYS_TICK_TMR_ID_GPS_TASK) )
	{
		return;
	}

	USART3_RX_STA |= 1<<15;
	USART3_RX_BUF[USART3_RX_STA & 0x7FFF] = 0;//Ìí¼Ó½áÊø·û

	memset(_search_buf, 0, sizeof(_search_buf));
	buf	= (uint8_t *)tool_get_str_gnrmc((char *)USART3_RX_BUF, (char *)_search_buf);
	if(buf != NULL)			
	{	
		if(tool_get_str_gnrmc_keyvalue((char *)_search_buf, (char *)str_ddmmyy, (char *)str_hhmmss, (char *)str_status, (char *)str_longitude, (char *)str_latitude, (char *)str_speed, (char *)str_dir) == 1)
		{
			val_status = tool_get_gps_status((char *)str_status);
			val_longitude = tool_convert_gps_longitude_latitude((char *)str_longitude);
			val_latitude = tool_convert_gps_longitude_latitude((char *)str_latitude);
			val_speed = tool_convert_gps_speed((char *)str_speed);
			val_dir = tool_convert_gps_ground_course((char *)str_dir);
			val_hhmmss = tool_convert_gps_hhmmss((char *)str_hhmmss);
			val_ddmmyy = tool_convert_gps_ddmmyy((char *)str_ddmmyy);

			CLI();
			g_status = val_status;
			g_longitude = val_longitude;
			g_latitude = val_latitude;
			g_speed = val_speed;
			g_course = val_dir;
			g_ddmmyy = val_ddmmyy;
			g_hhmmss = val_hhmmss;
			SEI();

			#if DEBUG_PRINTF		
			debug_printf("%d,%d,%d,%d,%d,%d,%d\r\n", g_status, g_longitude, g_latitude, g_speed, g_course, g_ddmmyy, g_hhmmss);
			#endif	
		}
	}
	#if DEBUG_PRINTF		
	debug_printf("%s\r\n", _search_buf);
	#endif	
	
	USART3_RX_STA = 0;
	memset(USART3_RX_BUF, 0, sizeof(USART3_RX_BUF));
	
	start_soft_tmr( SYS_TICK_TMR_ID_GPS_TASK, 2000 );
}


