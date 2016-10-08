
/*********************************************************************************
 *
 *			 														����ͨѶ��������
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

//���� USART1��USART3 �� printf ������ӡ�ķ��ͻ�����
static uint8_t 	  _usart1_send_buf[USART_BASE_LEN];
static uint8_t 	  _usart3_send_buf[USART_BASE_LEN];
static uint8_t 	  _usart4_send_buf[USART_BASE_LEN];
static uint8_t 	  _usart4_recv_buf[USART_BASE_LEN];

static usart_queue_t _usart4_recv_queue;

static uint8_t    _usart4_frame_buffer[USART_BASE_LEN];
static uint16_t   _usart4_frame_index = 0;
static uint16_t   _usart4_frame_length = 0;

uint16_t USART1_RX_STA = 0;
uint8_t USART1_RX_BUF[USART_BASE_LEN]; //���ջ���,���USART3_MAX_RECV_LEN���ֽ�.
uint16_t USART3_RX_STA = 0;
uint8_t USART3_RX_BUF[USART3_GPS_RECV_BUF]; //���ջ���,���USART3_MAX_RECV_LEN���ֽ�.


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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	//USART1_RX	  PB.7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
#else
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
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

	USART_ClearFlag(USART1, USART_FLAG_TC);//���USART1��1 ���ֽ��޷���ȷ���͵����� 	
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
	/*  ��USART1�����������ʱ��  */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	/*  ����TX��GPIOΪ���츴��ģʽ  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*  ����RX��GPIOΪ��������ģʽ  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*  ����USART ������Ӳ�������Ʋ�ʹ��  */
	USART_InitStructure.USART_BaudRate = 115200;					// ���ò�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;				// 8 λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					// ��֡��β����һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;						// ��ż��ʹ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			// ����ʹ�ܡ�����ʹ��
#else
	/*  ��USART1�����������ʱ��  */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/*  ����TX��GPIOΪ���츴��ģʽ  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*  ����RX��GPIOΪ��������ģʽ  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*  ����USART ������Ӳ�������Ʋ�ʹ��  */
	USART_InitStructure.USART_BaudRate = 9600;					// ���ò�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;				// 8 λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					// ��֡��β����һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;						// ��ż��ʹ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			// ����ʹ�ܡ�����ʹ��
#endif


	

	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART3, ENABLE);													// ������ϣ�ʹ��USART2
	
	USART_ClearFlag(USART3, USART_FLAG_TC);//���USART2��1 ���ֽ��޷���ȷ���͵�����     
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

	/*  ��USART1�����������ʱ��  */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	/*  ����TX��GPIOΪ���츴��ģʽ  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*  ����RX��GPIOΪ��������ģʽ  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*  ����USART ������Ӳ�������Ʋ�ʹ��  */
	USART_InitStructure.USART_BaudRate = 115200;					// ���ò�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;				// 8 λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_1;					// ��֡��β����һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;						// ��ż��ʹ��
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;			// ����ʹ�ܡ�����ʹ��

	USART_Init(UART4, &USART_InitStructure);

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(UART4, ENABLE);													// ������ϣ�ʹ��USART2
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
	i=strlen((const char*)_usart1_send_buf);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
	  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
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
	for(i=0;i<len;i++)							//ѭ����������
	{
	  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
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
	i = strlen((const char*)_usart3_send_buf);		//�˴η������ݵĳ���
	for(j=0;j<i;j++)							//ѭ����������
	{
	  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET); //ѭ������,ֱ���������   
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
		i = strlen((const char*)_usart4_send_buf);		//�˴η������ݵĳ���
		for(j=0;j<i;j++)							//ѭ����������
		{ 
			while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET); //ѭ������,ֱ���������   
			USART_SendData(UART4, _usart4_send_buf[j]); 
		} 
	}
#endif
}

static uint8_t _usart4_queue_get_byte(uint8_t *p_byte)
{
	uint16_t rx_write;
	usart_queue_t *p_queue = &_usart4_recv_queue;
								
	CLI();																							// �ر����ж�			
	rx_write = p_queue->rx_write;												// ��ȡ����дָ��λ��
	SEI();																							// �������ж�

	if (p_queue->rx_read == rx_write)										// ���ն���Ϊ�գ�û�����ݿɶ� 
	{
		return RET_FALSE;											
	}
	else
	{														
		*p_byte = p_queue->rx_buf[p_queue->rx_read];			// ������ݽ����ն���
		CLI();																						// �ر����ж�
		if (++p_queue->rx_read >= p_queue->rx_buf_size)		// ��ָ�뵽����ն���β
		{
			p_queue->rx_read = 0;														// ���ն��ж�ָ��ص�
		}
		SEI();																						// �������ж�		
		return RET_TRUE;
	}
}

/*******************************************************************************************
 * ����: �Ӷ����л�ȡһ֡����
 * ע��: �˴�ֻУ������֡ͷ���ͳ��ȣ�ֻ���𴮿�ͨѶ�Լ���·�����������		
 *			��������Ӧ�ò�Ľ����������� UsartTask ȥ��
*******************************************************************************************/
uint8_t usart4_recv_packet(uint8_t *buf, uint16_t *len)
{
	uint8_t          recv_byte = 0;
	uint16_t         frame_len = 0;

	//����������
	if(_usart4_queue_get_byte(&recv_byte) != RET_TRUE)
	{
		return RET_FALSE;
	}

	_usart4_frame_buffer[_usart4_frame_index++] = recv_byte;

	// �ж�����֡ͷ���Ƿ����
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

	//���ݳ����ж�
	if(_usart4_frame_index == 8)
	{
		//������ֲ����ע��˴��Ĵ�˺�С�˸�ʽ,ԭ���� STM32 �� VS �������ߵĴ��С�˸�ʽ��ͬ
		//STM32ΪС�˸�ʽ��VS������ô�˸�ʽ
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

	//�жϵ�ǰ���յ�������֡�����Ƿ񳬱�
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


//����һ֡����
void usart4_send_packet(uint8_t *p_buf, uint16_t len)
{
	uint16_t i;

	for (i=0; i<len; i++)
	{	
		while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
		USART_SendData(UART4, p_buf[i]); 
	}
}

static void _usart4_irq_msg_handler(usart_queue_t *p_queue)
{
	if(p_queue->rx_write == p_queue->rx_read) 										//���ն�������
	{
		if(p_queue->rx_byte_finish != null)
		{
			p_queue->rx_byte_finish();																//ִ�лص�����
		}
	}

	p_queue->rx_buf[p_queue->rx_write] = USART_ReceiveData(UART4);
	if(++p_queue->rx_write >= p_queue->rx_buf_size)								//дָ�뵽�����ѭ�����е�β��
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
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)//���յ�����
	{
		recv_byte = USART_ReceiveData(USART1);
		if((USART1_RX_STA & (1<<15)) == 0)//�������һ������,��û�б�����,���ٽ�����������
		{ 
			if(USART1_RX_STA < USART_BASE_LEN)	//�����Խ�������
			{
				USART1_RX_BUF[USART1_RX_STA++] = recv_byte;	//��¼���յ���ֵ	 
			}
			else 
			{
				USART1_RX_STA |= 1<<15;				//ǿ�Ʊ�ǽ������
			} 
		}
	}
} 

void USART3_IRQHandler(void)
{
	uint8_t recv_byte = 0;	
	static uint8_t status = 0;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
	{
		recv_byte = USART_ReceiveData(USART3);
		if((USART3_RX_STA & (1<<15)) == 0)//�������һ������,��û�б�����,���ٽ�����������
		{ 
			if(USART3_RX_STA < USART_BASE_LEN)	//�����Խ�������
			{
				USART3_RX_BUF[USART3_RX_STA++] = recv_byte;	//��¼���յ���ֵ	 
			}
			else 
			{
				USART3_RX_STA |= 1<<15;				//ǿ�Ʊ�ǽ������
			} 

			//�ж�֡ͷ
			if(USART3_RX_STA == 6)
			{
				if(strncmp((const char *)USART3_RX_BUF, "$GNRMC", 6) != 0)//֡ͷ����
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
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)//���յ�����
	{
		_usart4_irq_msg_handler(&_usart4_recv_queue);
	}
}

 
