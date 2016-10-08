#include "Tool.h"
#include "Usart.h"
#include <string.h>


char* tool_get_str_gnrmc(char* full_str, char *out_str)
{
	char *p_head;
	char *p_end;
	uint32_t _head_addr = 0;
	uint32_t _end_addr = 0;

	//��ʼ
	if( (p_head = strstr(full_str, "$GNRMC") )== NULL  )
	{
		return NULL;		
	}
	else
	{
		_head_addr = (uint32_t)(p_head + 7);
	}
	//����
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
		if(i == 0)//ʱ����
		{
			strncpy(hhmmss, pos, end_pos - pos);
		}
		else if(i == 1)//��ȡ��λ״̬
		{
			strncpy(status, pos, end_pos - pos);
		}
		else if(i == 2)//��ȡγ��
		{
			strncpy(longitude, pos, end_pos - pos);
		}
		else if(i == 4)//��ȡ����
		{
			strncpy(latitude, pos, end_pos - pos);
		}
		else if(i == 6)//��ȡ�ٶ�
		{
			strncpy(speed, pos, end_pos - pos);
		}
		else if(i == 7)//����
		{
			strncpy(dir, pos, end_pos - pos);
		}
		else if(i == 8)//������
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
	
	speed = (uint16_t)(float)(((float)uint_abcd + (float)((uint_fghi % 1000)/100)) * 18.52);//1�ڵ��� 1.852Km/h��Э������ĵ�λΪ 0.1Km/h

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
-- 	�� �� ��:	  tool_get_key_str
-- 	����������	��ȡ�ַ���Э���е�����
--  ��ڲ�����	longStr	:	Ҫ��ѯ�ĳ��ַ��� ����"{mac=00-11-22-33-44-55;ip=192.168.0.27;}"
--							keyStr	:	Ҫ��ȡ���ݵĹؼ��֣�����"ip"	
--							plen		:	��ȡ�����ݵĳ��ȣ����統keyStrΪ"ip"ʱ��*plen =12 
--  �� �� ֵ��	�ؼ��ֶ�Ӧ���ݵ���ʵλ�ã����磺"ip"��Ӧ�ķ���ֵΪ "192.168.0.27"�Ŀ�ʼλ��
-- 	˵    ���� 	������ֻ������¥��Э���"{keystr=content;}"�ĸ�ʽ
------------------------------------------------------------------------------------------------------------------------*/
char *tool_get_key_str(char *full_str, char* key_str, int *plen)
{
	char *start_pos, *end_pos;

	start_pos = strstr((const char *)full_str,(const char *)key_str);
	if( !start_pos )				
	{
		return NULL;		// ���ؼ����Ƿ����
	}
	start_pos = start_pos + strlen(key_str) ;			// �����ؼ���
	if( *start_pos++ != '=' )	
	{
		return NULL;		// ����������Ⱥ�=�ţ�				
	}
	end_pos = strstr(start_pos,";");					// ���ҽ�����
	if( !end_pos )				
	{
		return NULL;															
	}
	*plen = end_pos - start_pos;							// ���ֻ�ȡ����

	return start_pos;													// ����������ʼλ��
}


/*------------------------------------------------------------------------------------------------------------------------
-- 	�� �� ��:		tool_get_key_int
-- 	����������	��ȡ�ַ���Э���е�%d��ֵ
--  ��ڲ�����	longStr	:	Ҫ��ѯ�ĳ��ַ��� ����"{cardtime:%d;}"
--							keyStr	:	Ҫ��ȡ���ݵĹؼ��֣�����"cardtime"	
--  �� �� ֵ��	���ض�Ӧ������ֵ
--							������ֵΪһ���ܴ�ĸ�ֵ��ʱ��˵��û���ҵ�
-- 	˵    ���� 	������ֻ������¥��Э���"{keystr=content;}"�ĸ�ʽ
------------------------------------------------------------------------------------------------------------------------*/
u32 tool_get_key_int(char* full_str, char* key_str)
{
	char *p_str, *end_str, buf[12];
	u32 ret,len;

	p_str = tool_get_key_str(full_str, key_str, (int*)&len);	 	// �ҵ��ؼ���
	if( p_str == NULL)
	{
		return 0;															// �ж��Ƿ��и�ֵ
	}
	strncpy(buf, p_str, len);								// ������������
	*(buf + len) = 0;												// +����������ֹ�ں����д���
	ret = strtoul(buf, &end_str, 10); 			// �˴�������atoi���������

	return ret;
}



