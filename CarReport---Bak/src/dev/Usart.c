
/*********************************************************************************
 *
 *			 														串口通讯驱动程序
 *																  By : Tack.Fang
 *
 ********************************************************************************/
#include "Usart.h"
#include "Core.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "Gpio.h"
#include "Sim800.h"

uint8_t debug_state = 0;

//用于 USART1、USART3 的 printf 函数打印的发送缓冲区
static uint8_t 	  _usart1_send_buf[USART_BASE_LEN];
static uint8_t 	  _usart3_send_buf[USART_BASE_LEN];
static uint8_t 	  _usart4_send_buf[USART_BASE_LEN];
static uint8_t 	  _usart4_recv_buf[USART_BASE_LEN];

static usart_queue_t _usart4_recv_queue;

static uint8_t    _usart4_frame_buffer[USART_BASE_LEN];
static uint16_t   _usart4_frame_index = 0;
static uint16_t   _usart4_frame_length = 0;

uint16_t USART1_RX_STA = 0;
uint8_t USART1_RX_BUF[USART_BASE_LEN]; //接收缓冲,最大USART3_MAX_RECV_LEN个字节.
uint16_t USART3_RX_STA = 0;
uint8_t USART3_RX_BUF[USART3_GPS_RECV_BUF]; //接收缓冲,最大USART3_MAX_RECV_LEN个字节.


static void _init_usart1_cfg(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
#if USART1_REMAP_PB06_PB07_FOR_DEBUG
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	//USART1_TX   PB.6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //PB.6
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推免输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	//USART1_RX	  PB.7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
#else
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推免输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
#endif

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


  USART_Init(USART1, &USART_InitStructure);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
	USART_Cmd(USART1, ENABLE); 

	USART_ClearFlag(USART1, USART_FLAG_TC);//解决USART1第1 个字节无法正确发送的问题 	
}


static void _init_usart1_nvic(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
}


static void _init_usart3_cfg(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;

#if USART3_REMAP_PC10_PC11_FOR_DEBUG
	/*  打开USART1部件和外设的时钟  */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	/*  配置TX的GPIO为推挽复用模式  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*  配置RX的GPIO为浮空输入模式  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*  配置USART 参数，硬件流控制不使能  */
	USART_InitStructure.USART_BaudRate = 115200;					// 设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;				// 8 位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					// 在帧结尾传送一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;						// 奇偶不使能
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			// 接收使能、发送使能
#else
	/*  打开USART1部件和外设的时钟  */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/*  配置TX的GPIO为推挽复用模式  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*  配置RX的GPIO为浮空输入模式  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*  配置USART 参数，硬件流控制不使能  */
	USART_InitStructure.USART_BaudRate = 9600;					// 设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;				// 8 位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					// 在帧结尾传送一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;						// 奇偶不使能
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			// 接收使能、发送使能
#endif


	

	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART3, ENABLE);													// 配置完毕，使能USART2
	
	USART_ClearFlag(USART3, USART_FLAG_TC);//解决USART2第1 个字节无法正确发送的问题     
}


static void _init_usart3_nvic(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
}

static void _init_usart4_cfg(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;

	/*  打开USART1部件和外设的时钟  */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	/*  配置TX的GPIO为推挽复用模式  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*  配置RX的GPIO为浮空输入模式  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*  配置USART 参数，硬件流控制不使能  */
	USART_InitStructure.USART_BaudRate = 115200;					// 设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;				// 8 位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					// 在帧结尾传送一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;						// 奇偶不使能
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			// 接收使能、发送使能

	USART_Init(UART4, &USART_InitStructure);

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(UART4, ENABLE);													// 配置完毕，使能USART2
}


static void _init_usart4_nvic(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
}

static void _init_uart4_queue(void)
{
	_usart4_recv_queue.rx_buf	  = _usart4_recv_buf;
	_usart4_recv_queue.rx_buf_size = sizeof(_usart4_recv_buf);
	_usart4_recv_queue.rx_write		= 0;
	_usart4_recv_queue.rx_read		= 0;
	_usart4_recv_queue.rx_byte_finish = null;	
}



void usart1_printf(char* fmt, ...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)_usart1_send_buf, fmt, ap);
	va_end(ap);
	i=strlen((const char*)_usart1_send_buf);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
	  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
		USART_SendData(USART1, _usart1_send_buf[j]); 
	} 
}

void usart1_send_data(uint8_t *buf, uint16_t len)  
{  
	uint16_t i; 
	static uint8_t byte_status = 0;

	if(byte_status == 1)
	{
		byte_status = 0;
		CLR_GSM_SEND_FRAME();
	}
	else
	{
		byte_status = 1;
		SET_GSM_SEND_FRAME();
	}

	CLR_GSM_SEND_BYTE();
	for(i=0;i<len;i++)							//循环发送数据
	{
	  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
		USART_SendData(USART1, buf[i]); 
	} 
	SET_GSM_SEND_BYTE();
}


void usart3_printf(char* fmt, ...)  
{  
	u16 i, j; 
	va_list ap; 
	va_start(ap, fmt);
	vsprintf((char*)_usart3_send_buf, fmt, ap);
	va_end(ap);
	i = strlen((const char*)_usart3_send_buf);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
	  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET); //循环发送,直到发送完毕   
		USART_SendData(USART3, _usart3_send_buf[j]); 
	} 
}

void debug_printf(char* fmt, ...)  
{  
#if DEBUG_PRINTF
	u16 i, j; 
	va_list ap; 
	if(debug_state) 
	{
		va_start(ap, fmt);
		vsprintf((char*)_usart4_send_buf, fmt, ap);
		va_end(ap);
		i = strlen((const char*)_usart4_send_buf);		//此次发送数据的长度
		for(j=0;j<i;j++)							//循环发送数据
		{ 
			while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET); //循环发送,直到发送完毕   
			USART_SendData(UART4, _usart4_send_buf[j]); 
		} 
	}
#endif
}

static uint8_t _usart4_queue_get_byte(uint8_t *p_byte)
{
	uint16_t rx_write;
	usart_queue_t *p_queue = &_usart4_recv_queue;
								
	CLI();																							// 关闭总中断			
	rx_write = p_queue->rx_write;												// 获取接收写指针位置
	SEI();																							// 开启总中断

	if (p_queue->rx_read == rx_write)										// 接收队列为空，没有数据可读 
	{
		return RET_FALSE;											
	}
	else
	{														
		*p_byte = p_queue->rx_buf[p_queue->rx_read];			// 填充数据进接收队列
		CLI();																						// 关闭总中断
		if (++p_queue->rx_read >= p_queue->rx_buf_size)		// 读指针到达接收队列尾
		{
			p_queue->rx_read = 0;														// 接收队列读指针回调
		}
		SEI();																						// 开启总中断		
		return RET_TRUE;
	}
}

/*******************************************************************************************
 * 描述: 从队列中获取一帧数据
 * 注意: 此处只校验数据帧头部和长度，只负责串口通讯以及链路层的驱动程序，		
 *			其余网络应用层的解析工作丢给 UsartTask 去做
*******************************************************************************************/
uint8_t usart4_recv_packet(uint8_t *buf, uint16_t *len)
{
	uint8_t          recv_byte = 0;
	uint16_t         frame_len = 0;

	//读串口数据
	if(_usart4_queue_get_byte(&recv_byte) != RET_TRUE)
	{
		return RET_FALSE;
	}

	_usart4_frame_buffer[_usart4_frame_index++] = recv_byte;

	// 判断数据帧头部是否相符
	if(_usart4_frame_index == 6 ) 
	{
		if(strncmp((const char *)_usart4_frame_buffer, "baiyue", 6) != 0)
		{
			_usart4_frame_index = 0;	
			_usart4_frame_length = 0;
			memset(_usart4_frame_buffer, 0, sizeof(_usart4_frame_buffer));
			return RET_FALSE;
		}
	}

	//数据长度判断
	if(_usart4_frame_index == 8)
	{
		//如有移植，请注意此处的大端和小端格式,原因是 STM32 和 VS 升级工具的大端小端格式不同
		//STM32为小端格式，VS软件采用大端格式
		frame_len = _usart4_frame_buffer[FRAME_LEN_1_POS] & 0x00FF;
		frame_len = frame_len << 8;
		frame_len = frame_len + _usart4_frame_buffer[FRAME_LEN_0_POS];

		if(frame_len > USART_BASE_LEN)
		{
			_usart4_frame_index = 0;	
			_usart4_frame_length = 0;
			memset(_usart4_frame_buffer, 0, sizeof(_usart4_frame_buffer));
			debug_printf("%s---> frame_len %d > 1024\r\n", __func__, frame_len);
			return RET_FALSE;
		}
		_usart4_frame_length = frame_len;
	}

	//判断当前接收到的数据帧长度是否超标
	if(_usart4_frame_index > USART_BASE_LEN)
	{
		_usart4_frame_index = 0;	
		_usart4_frame_length = 0;
		memset(_usart4_frame_buffer, 0, sizeof(_usart4_frame_buffer));
		debug_printf("%s---> _usart4_frame_index %d > 1024\r\n", __func__, _usart4_frame_index);
		return RET_FALSE;
	}

	if( _usart4_frame_index == _usart4_frame_length)
	{
		if(_usart4_frame_index < FRAME_HEAD_LEN)
		{
			_usart4_frame_index = 0;	
			_usart4_frame_length = 0;
			memset(_usart4_frame_buffer, 0, sizeof(_usart4_frame_buffer));
			debug_printf("%s---> _usart4_frame_index < 26\r\n", __func__);
			return RET_FALSE;
		}
		else
		{
			*len = _usart4_frame_length;
			memcpy(buf, _usart4_frame_buffer, _usart4_frame_length);

			_usart4_frame_index = 0;	
			_usart4_frame_length = 0;
			memset(_usart4_frame_buffer, 0, sizeof(_usart4_frame_buffer));
			return RET_TRUE;
		}
	}

	return RET_FALSE;
}


//发送一帧数据
void usart4_send_packet(uint8_t *p_buf, uint16_t len)
{
	uint16_t i;

	for (i=0; i<len; i++)
	{	
		while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
		USART_SendData(UART4, p_buf[i]); 
	}
}

static void _usart4_irq_msg_handler(usart_queue_t *p_queue)
{
	if(p_queue->rx_write == p_queue->rx_read) 										//接收队列满了
	{
		if(p_queue->rx_byte_finish != null)
		{
			p_queue->rx_byte_finish();																//执行回调函数
		}
	}

	p_queue->rx_buf[p_queue->rx_write] = USART_ReceiveData(UART4);
	if(++p_queue->rx_write >= p_queue->rx_buf_size)								//写指针到达接收循环队列的尾部
	{
		p_queue->rx_write = 0;
	}
}



void init_usart(void)
{
	_init_uart4_queue();
	
	_init_usart1_cfg();
	_init_usart3_cfg();
	_init_usart4_cfg();
	_init_usart1_nvic();
	_init_usart3_nvic();
	_init_usart4_nvic();
}

void USART1_IRQHandler(void)
{
	uint8_t recv_byte = 0;	      
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//接收到数据
	{
		recv_byte = USART_ReceiveData(USART1);
		if((USART1_RX_STA & (1<<15)) == 0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
			if(USART1_RX_STA < USART_BASE_LEN)	//还可以接收数据
			{
				USART1_RX_BUF[USART1_RX_STA++] = recv_byte;	//记录接收到的值	 
			}
			else 
			{
				USART1_RX_STA |= 1<<15;				//强制标记接收完成
			} 
		}
	}
} 

void USART3_IRQHandler(void)
{
	uint8_t recv_byte = 0;	
	static uint8_t status = 0;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//接收到数据
	{
		recv_byte = USART_ReceiveData(USART3);
		if((USART3_RX_STA & (1<<15)) == 0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
			if(USART3_RX_STA < USART_BASE_LEN)	//还可以接收数据
			{
				USART3_RX_BUF[USART3_RX_STA++] = recv_byte;	//记录接收到的值	 
			}
			else 
			{
				USART3_RX_STA |= 1<<15;				//强制标记接收完成
			} 

			//判断帧头
			if(USART3_RX_STA == 6)
			{
				if(strncmp((const char *)USART3_RX_BUF, "$GNRMC", 6) != 0)//帧头错误
				{
					memset(USART3_RX_BUF, 0, sizeof(USART3_RX_BUF));
					USART3_RX_STA = 0;

					if(status == 0)
					{
						SET_GPS_RECV_DATA();
						status = 1;
					}
					else
					{
						CLR_GPS_RECV_DATA();
						status = 0;
					}
				}
				else
				{
					
				}
			}
		}
	}
}

void UART4_IRQHandler(void) 
{
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)//接收到数据
	{
		_usart4_irq_msg_handler(&_usart4_recv_queue);
	}
}

 
