#include "Tool.h"
#include "Usart.h"
#include <string.h>


char* tool_get_str_gnrmc(char* full_str, char *out_str)
{
	char *p_head;
	char *p_end;
	uint32_t _head_addr = 0;
	uint32_t _end_addr = 0;

	//起始
	if( (p_head = strstr(full_str, "$GNRMC") )== NULL  )
	{
		return NULL;		
	}
	else
	{
		_head_addr = (uint32_t)(p_head + 7);
	}
	//结束
	if( (p_end = strstr(p_head + 7, "\n") )== NULL  )
	{
		return NULL;		
	}
	else
	{
		_end_addr = (uint32_t)p_end;
	}

	if(_end_addr <= _head_addr)
	{
		return NULL;
	}
	
	strncpy((char *)out_str, (char *)_head_addr, _end_addr - _head_addr);
	out_str[_end_addr - _head_addr] = 0;

	return out_str;								
}


uint8_t tool_get_str_gnrmc_keyvalue(char *str_gnrmc, char *ddmmyy, char *hhmmss, char *status, char *longitude, char *latitude, char *speed, char *dir)
{
	char i;
	char *pos = str_gnrmc, *end_pos;

	if( str_gnrmc == NULL)
	{
		return 0;
	}

  for( i=0; i<11; i++ )
	{
		if(( end_pos = strstr(pos, ",")) == NULL ) 
		{	
			return 0;
		}
		if(i == 0)//时分秒
		{
			strncpy(hhmmss, pos, end_pos - pos);
		}
		else if(i == 1)//获取定位状态
		{
			strncpy(status, pos, end_pos - pos);
		}
		else if(i == 2)//获取纬度
		{
			strncpy(longitude, pos, end_pos - pos);
		}
		else if(i == 4)//获取经度
		{
			strncpy(latitude, pos, end_pos - pos);
		}
		else if(i == 6)//获取速度
		{
			strncpy(speed, pos, end_pos - pos);
		}
		else if(i == 7)//方向
		{
			strncpy(dir, pos, end_pos - pos);
		}
		else if(i == 8)//年月日
		{
			strncpy(ddmmyy, pos, end_pos - pos);
		}
		pos = end_pos + 1;
  }

	return 1;
}

uint8_t tool_get_gps_status(char *status)
{
	char *pos = status;
	
	if(strstr(pos, "A") != NULL ) 
	{	
		return 0;
	}
	else if(strstr(pos, "V") != NULL ) 
	{	
		return 1;
	}

	return 0;
}

uint32_t tool_convert_gps_longitude_latitude(char *input)
{
	char *pos = input, *end_pos;
	char str_abcde[6];
	char str_fghi[5];

	double abc;
	double de;
	double fghi;

	uint32_t data_abcde;
	uint32_t data_fghi;

	uint32_t result = 0;
	
	if( input == NULL)
	{
		return 0;
	}

	if(( end_pos = strstr(pos, ".")) == NULL ) 
	{	
		return 0;
	}

	strncpy(str_abcde, pos, end_pos - pos);
	str_abcde[end_pos - pos] = 0;
	strncpy(str_fghi, end_pos+1, 4);
	str_fghi[4] = 0;

	data_abcde = (uint32_t) atol(str_abcde);
	data_fghi = (uint32_t) atol(str_fghi);
	
	abc = data_abcde / 100;
	de = data_abcde % 100;
	fghi = data_fghi;

	result = abc * 1000000 + ((uint32_t)(de * 100000) + (uint32_t)(fghi * 10)) / 6;

	return result;
}


uint16_t tool_convert_gps_speed(char *input)
{
	char *pos = input, *end_pos;
	char str_abcd[5];
	char str_fghi[5];
	uint16_t uint_abcd = 0;
	uint16_t uint_fghi = 0;
	uint16_t speed = 0;

	if( input == NULL)
	{
		return 0;
	}

	if(( end_pos = strstr(pos, ".")) == NULL ) 
	{	
		return 0;
	}

	strncpy(str_abcd, pos, end_pos - pos);
	str_abcd[end_pos - pos] = 0;
	strncpy(str_fghi, end_pos+1, 4);
	str_fghi[4] = 0;

	uint_abcd = (uint16_t) atoi(str_abcd);
	uint_fghi = (uint16_t) atoi(str_fghi);
	
	speed = (uint16_t)(float)(((float)uint_abcd + (float)((uint_fghi % 1000)/100)) * 18.52);//1节等于 1.852Km/h，协议里面的单位为 0.1Km/h

	return speed;
}

uint16_t tool_convert_gps_ground_course(char *input)
{
	char *pos = input, *end_pos;
	char str_abcd[5];
	uint16_t course = 0;

	if( input == NULL)
	{
		return 0;
	}

	if(( end_pos = strstr(pos, ".")) == NULL ) 
	{	
		return 0;
	}

	strncpy(str_abcd, pos, end_pos - pos);
	str_abcd[end_pos - pos] = 0;

	course = (uint16_t) atoi(str_abcd);

	return course;
}


uint32_t tool_convert_gps_ddmmyy(char *input)
{
	uint32_t ddmmyy = 0;

	if( input == NULL)
	{
		return 0;
	}

	ddmmyy = (uint32_t) atoi(input);

	return ddmmyy;
}



uint32_t tool_convert_gps_hhmmss(char *input)
{
	char *pos = input, *end_pos;
	char str_abcd[8];
	uint32_t hhmmss = 0;

	if( input == NULL)
	{
		return 0;
	}

	if(( end_pos = strstr(pos, ".")) == NULL ) 
	{	
		return 0;
	}

	strncpy(str_abcd, pos, end_pos - pos);
	str_abcd[end_pos - pos] = 0;

	hhmmss = (uint32_t) atoi(str_abcd);

	return hhmmss;
}

uint16_t cal_crc16(uint8_t *buf, uint16_t len)
{
 	uint16_t i;
 	uint16_t crc16val = 0;

 	for (i = 0; i < len; i++)
  {
      crc16val += buf[i];
  }
 	return crc16val;
}


/*------------------------------------------------------------------------------------------------------------------------
-- 	函 数 名:	  tool_get_key_str
-- 	功能描述：	获取字符串协议中的内容
--  入口参数：	longStr	:	要查询的长字符串 比如"{mac=00-11-22-33-44-55;ip=192.168.0.27;}"
--							keyStr	:	要获取内容的关键字，比如"ip"	
--							plen		:	获取到内容的长度，比如当keyStr为"ip"时，*plen =12 
--  返 回 值：	关键字对应内容的其实位置，比如："ip"对应的返回值为 "192.168.0.27"的开始位置
-- 	说    明： 	本函数只适用于楼宇协议的"{keystr=content;}"的格式
------------------------------------------------------------------------------------------------------------------------*/
char *tool_get_key_str(char *full_str, char* key_str, int *plen)
{
	char *start_pos, *end_pos;

	start_pos = strstr((const char *)full_str,(const char *)key_str);
	if( !start_pos )				
	{
		return NULL;		// 检查关键字是否存在
	}
	start_pos = start_pos + strlen(key_str) ;			// 跳过关键字
	if( *start_pos++ != '=' )	
	{
		return NULL;		// 检验比跳过等号=号，				
	}
	end_pos = strstr(start_pos,";");					// 查找结束符
	if( !end_pos )				
	{
		return NULL;															
	}
	*plen = end_pos - start_pos;							// 保持获取长度

	return start_pos;													// 保持内容起始位置
}


/*------------------------------------------------------------------------------------------------------------------------
-- 	函 数 名:		tool_get_key_int
-- 	功能描述：	获取字符串协议中的%d的值
--  入口参数：	longStr	:	要查询的长字符串 比如"{cardtime:%d;}"
--							keyStr	:	要获取内容的关键字，比如"cardtime"	
--  返 回 值：	返回对应的整形值
--							当返回值为一个很大的负值得时候，说明没有找到
-- 	说    明： 	本函数只适用于楼宇协议的"{keystr=content;}"的格式
------------------------------------------------------------------------------------------------------------------------*/
u32 tool_get_key_int(char* full_str, char* key_str)
{
	char *p_str, *end_str, buf[12];
	u32 ret,len;

	p_str = tool_get_key_str(full_str, key_str, (int*)&len);	 	// 找到关键字
	if( p_str == NULL)
	{
		return 0;															// 判断是否有该值
	}
	strncpy(buf, p_str, len);								// 拷贝数字内容
	*(buf + len) = 0;												// +结束符，防止在函数中错误
	ret = strtoul(buf, &end_str, 10); 			// 此处不能用atoi会引起溢出

	return ret;
}



