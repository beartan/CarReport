
#include "Can.h"
#include "Usart.h"
#include "SysTick.h"
#include "Core.h"

uint8_t _car_vin_tmp[17];
uint8_t _car_vin[17];//�����������ݣ���֤����ͬ��

hy_can_data_t _hy_can_data;
ax_can_data_t _ax_can_data;
xx_can_data_t _xx_can_data;
cgqc_can_data_t _cgqc_can_data;
dykj_can_data_t _dykj_can_data;
jsyt_can_data_t _jsyt_can_data;
vict_can_data_t _vict_can_data;
ydzy_can_data_t _ydzy_can_data;

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:1~3; CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.��Χ:1~8;
//tbs1:ʱ���1��ʱ�䵥Ԫ.��Χ:1~16;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
//ע�����ϲ����κ�һ����������Ϊ0,�������.
//������=Fpclk1/((tsjw+tbs1+tbs2)*brp);
//mode:0,��ͨģʽ;1,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Normal_Init(1,8,7,5,1);
//������Ϊ:36M/((1+8+7)*9)=250Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��;

uint8_t init_can(void)
{

	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
 	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN_RX0_INT_ENABLE 
  NVIC_InitTypeDef  NVIC_InitStructure;
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);		//��ʼ��IO
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��IO
	  
 	//CAN��Ԫ����
 	CAN_InitStructure.CAN_TTCM = DISABLE;						 //��ʱ�䴥��ͨ��ģʽ  //
 	CAN_InitStructure.CAN_ABOM = DISABLE;						 //����Զ����߹���	 //
	CAN_InitStructure.CAN_AWUM = DISABLE;						 //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)//
	CAN_InitStructure.CAN_NART = ENABLE;						 	//��ֹ�����Զ����� //
	CAN_InitStructure.CAN_RFLM = DISABLE;						 //���Ĳ�����,�µĸ��Ǿɵ� // 
	CAN_InitStructure.CAN_TXFP = DISABLE;						 //���ȼ��ɱ��ı�ʶ������ //
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	         //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; //
	//���ò�����
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;//Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~ CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler = 9;            //��Ƶϵ��(Fdiv)Ϊbrp+1	36M/((1+8+7)*9)=250Kbps
	CAN_Init(CAN1, &CAN_InitStructure);            // ��ʼ��CAN1 

 	CAN_FilterInitStructure.CAN_FilterNumber = 0;	  //������0
 	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;////32λID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;//������0������FIFO0
 	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //���������0

	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��

#if CAN_RX0_INT_ENABLE
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

	memset(&_hy_can_data, 0 ,sizeof(_hy_can_data));
	memset(&_ax_can_data, 0, sizeof(_ax_can_data));

	return 0;
}   

void _handle_msg_car_vin(CanRxMsg *RxMessage)
{
	uint8_t i = 0;
	uint16_t check_sum = 0x0000;

	#if DEBUG_PRINTF
		debug_printf("%s\r\n", __func__);
	#endif

	for(i=0;i<8;i++)
	{
		#if DEBUG_PRINTF	
		debug_printf("%X ", RxMessage->Data[i]);
		#endif
	}

	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif

	for(i=0;i<7;i++)
	{
		check_sum += RxMessage->Data[i];
	}
	check_sum = check_sum & 0x00FF;
	
	if(check_sum != RxMessage->Data[7])
	{
		#if DEBUG_PRINTF	
		debug_printf("%s:%X != %X\r\n", __func__, check_sum, RxMessage->Data[7]);//0
		#endif
		
		return;
	}

	switch(RxMessage->Data[0])
	{
		case 0:
			memset(_car_vin_tmp, 0, sizeof(_car_vin_tmp));
			_car_vin_tmp[0x00] = RxMessage->Data[1];
			_car_vin_tmp[0x01] = RxMessage->Data[2];
			_car_vin_tmp[0x02] = RxMessage->Data[3];
			_car_vin_tmp[0x03] = RxMessage->Data[4];
			_car_vin_tmp[0x04] = RxMessage->Data[5];
			_car_vin_tmp[0x05] = RxMessage->Data[6];
			break;
		case 1:
			_car_vin_tmp[0x06] = RxMessage->Data[1];
			_car_vin_tmp[0x07] = RxMessage->Data[2];
			_car_vin_tmp[0x08] = RxMessage->Data[3];
			_car_vin_tmp[0x09] = RxMessage->Data[4];
			_car_vin_tmp[0x0A] = RxMessage->Data[5];
			_car_vin_tmp[0x0B] = RxMessage->Data[6];
			break;
		case 2:
			_car_vin_tmp[0x0C] = RxMessage->Data[1];
			_car_vin_tmp[0x0D] = RxMessage->Data[2];
			_car_vin_tmp[0x0E] = RxMessage->Data[3];
			_car_vin_tmp[0x0F] = RxMessage->Data[4];
			_car_vin_tmp[0x10] = RxMessage->Data[5];
			memset(_car_vin, 0, sizeof(_car_vin));
			memcpy(_car_vin, _car_vin_tmp, sizeof(_car_vin_tmp));
			break;
			
		default:
			break;
	}
}

void _handle_msg_mileage(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif
	
	for(i=0;i<6;i++)
	{
		_hy_can_data._mileage[i] = RxMessage->Data[i];
	
		#if DEBUG_PRINTF	
			debug_printf("%X ", _hy_can_data._mileage[i]);
		#endif
	}
	
	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}


void _handle_msg_mileage2(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif

	for(i=0;i<2;i++)
	{
		_hy_can_data._mileage[i+6] = RxMessage->Data[i];
	
		#if DEBUG_PRINTF	
			debug_printf("%X ", _hy_can_data._mileage[i]);
		#endif
	}
	
	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}


void _handle_msg_motor_speed(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif
	
	for(i=0;i<8;i++)
	{
		_hy_can_data._motor_speed[i] = RxMessage->Data[i];

		#if DEBUG_PRINTF	
		debug_printf("%X ", _hy_can_data._motor_speed[i]);
		#endif
	}

	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}

void _handle_msg_battery(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif
	
	for(i=0;i<8;i++)
	{
		_hy_can_data._battery[i] = RxMessage->Data[i];

		#if DEBUG_PRINTF	
		debug_printf("%X ", _hy_can_data._battery[i]);
		#endif
	}
	
	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}

void _handle_msg_car_status(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif
	
	for(i=0;i<8;i++)
	{
		_hy_can_data._car_status[i] = RxMessage->Data[i];

		#if DEBUG_PRINTF	
		debug_printf("%X ", _hy_can_data._car_status[i]);
		#endif
	}

	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}

void _handle_msg_motor_torque(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif
	
	for(i=0;i<8;i++)
	{
		_hy_can_data._motor_torque[i] = RxMessage->Data[i];

		#if DEBUG_PRINTF	
		debug_printf("%X ", _hy_can_data._motor_torque[i]);
		#endif
	}

	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}

void _handle_msg_motor_temp(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif
	
	for(i=0;i<8;i++)
	{
		_hy_can_data._motor_temp[i] = RxMessage->Data[i];

		#if DEBUG_PRINTF	
		debug_printf("%X ", _hy_can_data._motor_temp[i]);
		#endif
	}

	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}


void _handle_msg_motor_current(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif

	for(i=0;i<8;i++)
	{
		_hy_can_data._motor_current[i] = RxMessage->Data[i];

		#if DEBUG_PRINTF	
		debug_printf("%X ", _hy_can_data._motor_current[i]);
		#endif
	}

	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}

void _handle_msg_charge_voltage(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif
	
	for(i=0;i<8;i++)
	{
		_hy_can_data._charge_voltage[i] = RxMessage->Data[i];

		#if DEBUG_PRINTF	
		debug_printf("%X ", _hy_can_data._charge_voltage[i]);
		#endif
	}

	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}

void _handle_msg_resistance(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif
	
	for(i=0;i<8;i++)
	{
		_hy_can_data._resistance[i] = RxMessage->Data[i];

		#if DEBUG_PRINTF	
		debug_printf("%X ", _hy_can_data._resistance[i]);
		#endif
	}

	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}

void _handle_msg_error_alarm(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	#if DEBUG_PRINTF	
		debug_printf("%s\r\n", __func__);
	#endif
	
	for(i=0;i<8;i++)
	{
		_hy_can_data._error_alarm[i] = RxMessage->Data[i];

		#if DEBUG_PRINTF	
		debug_printf("%X ", _hy_can_data._error_alarm[i]);
		#endif
	}

	#if DEBUG_PRINTF	
	debug_printf("\r\n");
	#endif
}


//������AX����Э����
void _ax_handle_msg_01(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<7;i++)
	{
		_ax_can_data._data_01[i] = RxMessage->Data[i];
	}
}

void _ax_handle_msg_02(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<4;i++)
	{
		_ax_can_data._data_02[i] = RxMessage->Data[i];
	}
}

void _ax_handle_msg_03(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<8;i++)
	{
		_ax_can_data._data_03[i] = RxMessage->Data[i];
	}
}

void _ax_handle_msg_04(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<8;i++)
	{
		_ax_can_data._data_04[i] = RxMessage->Data[i];
	}
}

void _ax_handle_msg_05(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<1;i++)
	{
		_ax_can_data._data_05[i] = RxMessage->Data[i];
	}
}

//ID=0x0C11D0F3
void _ax_handle_msg_06(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<8;i++)
	{
		_ax_can_data._data_06[i] = RxMessage->Data[i];
	}
}

//0x0C20D0F3
void _ax_handle_msg_09(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<4;i++)
	{
		_ax_can_data._data_09[i] = RxMessage->Data[i];
	}
}

//0x0C21D0F3
void _ax_handle_msg_10(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<8;i++)
	{
		_ax_can_data._data_10[i] = RxMessage->Data[i];
	}
}

//0x0C22D0F3
void _ax_handle_msg_11(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<8;i++)
	{
		_ax_can_data._data_11[i] = RxMessage->Data[i];
	}
}

//0x0C30D0F3
void _ax_handle_msg_12(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<5;i++)
	{
		_ax_can_data._data_12[i] = RxMessage->Data[i];
	}
}

//0x0C31D0F3
void _ax_handle_msg_13(CanRxMsg *RxMessage)
{
	uint8_t i = 0;

	for(i=0;i<4;i++)
	{
		_ax_can_data._data_13[i] = RxMessage->Data[i];
	}
}

void _ax_handle_msg_14(CanRxMsg *RxMessage)
{
	uint8_t i = 0;
	
	for(i=0;i<8;i++)
	{
		_ax_can_data._data_14[i] = RxMessage->Data[i];
	}
}

void _ax_handle_msg_15(CanRxMsg *RxMessage)
{
	uint8_t i = 0;
	
	for(i=0;i<6;i++)
	{
		_ax_can_data._data_15[i] = RxMessage->Data[i];
	}
}

void _ax_handle_msg_16(CanRxMsg *RxMessage)
{
	uint8_t i = 0;
	
	for(i=0;i<5;i++)
	{
		_ax_can_data._data_16[i] = RxMessage->Data[i];
	}
}

void _xx_handle_msg_01(CanRxMsg *RxMessage)
{
	_xx_can_data._data_01[0] = RxMessage->Data[0];
	_xx_can_data._data_01[1] = RxMessage->Data[1];
	_xx_can_data._data_01[2] = RxMessage->Data[2];
	_xx_can_data._data_01[3] = RxMessage->Data[3];
	_xx_can_data._data_01[4] = RxMessage->Data[4];
	_xx_can_data._data_01[5] = RxMessage->Data[6];
	_xx_can_data._data_01[6] = RxMessage->Data[7];
}

void _xx_handle_msg_02(CanRxMsg *RxMessage)
{
	_xx_can_data._data_02[0] = RxMessage->Data[0];
	_xx_can_data._data_02[1] = RxMessage->Data[1];
	_xx_can_data._data_02[2] = RxMessage->Data[2];
	_xx_can_data._data_02[3] = RxMessage->Data[3];
	_xx_can_data._data_02[4] = RxMessage->Data[4];
	_xx_can_data._data_02[5] = RxMessage->Data[5];
	_xx_can_data._data_02[6] = RxMessage->Data[7];
}

void _xx_handle_msg_03(CanRxMsg *RxMessage)
{
	uint8_t i = 0;
	
	for(i=0;i<6;i++)
	{
		_xx_can_data._data_03[i] = RxMessage->Data[i];
	}
}

void _xx_handle_msg_04(CanRxMsg *RxMessage)
{
	uint8_t i = 0;
	
	for(i=0;i<7;i++)
	{
		_xx_can_data._data_04[i] = RxMessage->Data[i];
	}
}

void _xx_handle_msg_05(CanRxMsg *RxMessage)
{
	_xx_can_data._data_05[0] = RxMessage->Data[0];
	_xx_can_data._data_05[1] = RxMessage->Data[1];
	_xx_can_data._data_05[2] = RxMessage->Data[2];
	_xx_can_data._data_05[3] = RxMessage->Data[3];
	_xx_can_data._data_05[4] = RxMessage->Data[5];
	_xx_can_data._data_05[5] = RxMessage->Data[6];
	_xx_can_data._data_05[6] = RxMessage->Data[7];
}

void _xx_handle_msg_06(CanRxMsg *RxMessage)
{
	_xx_can_data._data_06[0] = RxMessage->Data[2];
	_xx_can_data._data_06[1] = RxMessage->Data[5];
}

void _xx_handle_msg_07(CanRxMsg *RxMessage)
{
	_xx_can_data._data_07[0] = RxMessage->Data[2];
	_xx_can_data._data_07[1] = RxMessage->Data[3];
}

void _xx_handle_msg_08(CanRxMsg *RxMessage)
{
	uint8_t i = 0;
	
	for(i=0;i<7;i++)
	{
		_xx_can_data._data_08[i] = RxMessage->Data[i];
	}
}

void _xx_handle_msg_09(CanRxMsg *RxMessage)
{
	uint8_t i = 0;
	
	for(i=0;i<6;i++)
	{
		_xx_can_data._data_09[i] = RxMessage->Data[i];
	}
}

void _xx_handle_msg_10(CanRxMsg *RxMessage)
{
	_xx_can_data._data_10[0] = RxMessage->Data[0];
	_xx_can_data._data_10[1] = RxMessage->Data[2];
	_xx_can_data._data_10[2] = RxMessage->Data[3];
	_xx_can_data._data_10[3] = RxMessage->Data[4];
}

void _xx_handle_msg_11(CanRxMsg *RxMessage)
{
	uint8_t i = 0;
	
	for(i=0;i<4;i++)
	{
		_xx_can_data._data_11[i] = RxMessage->Data[i];
	}
}

void _xx_handle_msg_12(CanRxMsg *RxMessage)
{
	_xx_can_data._data_12[0] = RxMessage->Data[4];
	_xx_can_data._data_12[1] = RxMessage->Data[5];
}

void _cgqc_handle_msg_01(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_01[0] = RxMessage->Data[0];
	_cgqc_can_data._data_01[1] = RxMessage->Data[1];
	_cgqc_can_data._data_01[2] = RxMessage->Data[2];
	_cgqc_can_data._data_01[3] = RxMessage->Data[3];
	_cgqc_can_data._data_01[4] = RxMessage->Data[4];
	_cgqc_can_data._data_01[5] = RxMessage->Data[5];
	_cgqc_can_data._data_01[6] = RxMessage->Data[6];
	_cgqc_can_data._data_01[7] = RxMessage->Data[7];
}

void _cgqc_handle_msg_02(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_02[0] = RxMessage->Data[0];
	_cgqc_can_data._data_02[1] = RxMessage->Data[1];
	_cgqc_can_data._data_02[2] = RxMessage->Data[2];
	_cgqc_can_data._data_02[3] = RxMessage->Data[3];
	_cgqc_can_data._data_02[4] = RxMessage->Data[4];
	_cgqc_can_data._data_02[5] = RxMessage->Data[5];
	_cgqc_can_data._data_02[6] = RxMessage->Data[6];
	_cgqc_can_data._data_02[7] = RxMessage->Data[7];
}

void _cgqc_handle_msg_03(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_03[0] = RxMessage->Data[0];
	_cgqc_can_data._data_03[1] = RxMessage->Data[1];
	_cgqc_can_data._data_03[2] = RxMessage->Data[2];
	_cgqc_can_data._data_03[3] = RxMessage->Data[3];
	_cgqc_can_data._data_03[4] = RxMessage->Data[4];
	_cgqc_can_data._data_03[5] = RxMessage->Data[5];
	_cgqc_can_data._data_03[6] = RxMessage->Data[6];
	_cgqc_can_data._data_03[7] = RxMessage->Data[7];
}

void _cgqc_handle_msg_04(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_04[0] = RxMessage->Data[0];
	_cgqc_can_data._data_04[1] = RxMessage->Data[1];
	_cgqc_can_data._data_04[2] = RxMessage->Data[2];
	_cgqc_can_data._data_04[3] = RxMessage->Data[3];
	_cgqc_can_data._data_04[4] = RxMessage->Data[4];
	_cgqc_can_data._data_04[5] = RxMessage->Data[5];
	_cgqc_can_data._data_04[6] = RxMessage->Data[6];
	_cgqc_can_data._data_04[7] = RxMessage->Data[7];
}

void _cgqc_handle_msg_05(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_05[0] = RxMessage->Data[0];
	_cgqc_can_data._data_05[1] = RxMessage->Data[1];
	_cgqc_can_data._data_05[2] = RxMessage->Data[2];
	_cgqc_can_data._data_05[3] = RxMessage->Data[3];
	_cgqc_can_data._data_05[4] = RxMessage->Data[4];
	_cgqc_can_data._data_05[5] = RxMessage->Data[5];
	_cgqc_can_data._data_05[6] = RxMessage->Data[6];
	_cgqc_can_data._data_05[7] = RxMessage->Data[7];
}

void _cgqc_handle_msg_06(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_06[0] = RxMessage->Data[0];
	_cgqc_can_data._data_06[1] = RxMessage->Data[1];
	_cgqc_can_data._data_06[2] = RxMessage->Data[2];
	_cgqc_can_data._data_06[3] = RxMessage->Data[3];
	_cgqc_can_data._data_06[4] = RxMessage->Data[4];
	_cgqc_can_data._data_06[5] = RxMessage->Data[5];
	_cgqc_can_data._data_06[6] = RxMessage->Data[6];
	_cgqc_can_data._data_06[7] = RxMessage->Data[7];
}

void _cgqc_handle_msg_07(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_07[0] = RxMessage->Data[0];
	_cgqc_can_data._data_07[1] = RxMessage->Data[1];
	_cgqc_can_data._data_07[2] = RxMessage->Data[2];
	_cgqc_can_data._data_07[3] = RxMessage->Data[3];
	_cgqc_can_data._data_07[4] = RxMessage->Data[4];
	_cgqc_can_data._data_07[5] = RxMessage->Data[5];
	_cgqc_can_data._data_07[6] = RxMessage->Data[6];
	_cgqc_can_data._data_07[7] = RxMessage->Data[7];
}

void _cgqc_handle_msg_08(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_08[0] = RxMessage->Data[0];
	_cgqc_can_data._data_08[1] = RxMessage->Data[1];
	_cgqc_can_data._data_08[2] = RxMessage->Data[2];
	_cgqc_can_data._data_08[3] = RxMessage->Data[3];
	_cgqc_can_data._data_08[4] = RxMessage->Data[4];
	_cgqc_can_data._data_08[5] = RxMessage->Data[5];
	_cgqc_can_data._data_08[6] = RxMessage->Data[6];
	_cgqc_can_data._data_08[7] = RxMessage->Data[7];
}

void _cgqc_handle_msg_09(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_09[0] = RxMessage->Data[0];
	_cgqc_can_data._data_09[1] = RxMessage->Data[1];
	_cgqc_can_data._data_09[2] = RxMessage->Data[2];
	_cgqc_can_data._data_09[3] = RxMessage->Data[3];
	_cgqc_can_data._data_09[4] = RxMessage->Data[4];
	_cgqc_can_data._data_09[5] = RxMessage->Data[5];
	_cgqc_can_data._data_09[6] = RxMessage->Data[6];
	_cgqc_can_data._data_09[7] = RxMessage->Data[7];
}

void _cgqc_handle_msg_10(CanRxMsg *RxMessage)
{
	_cgqc_can_data._data_10[0] = RxMessage->Data[0];
	_cgqc_can_data._data_10[1] = RxMessage->Data[1];
	_cgqc_can_data._data_10[2] = RxMessage->Data[2];
	_cgqc_can_data._data_10[3] = RxMessage->Data[3];
	_cgqc_can_data._data_10[4] = RxMessage->Data[4];
	_cgqc_can_data._data_10[5] = RxMessage->Data[5];
	_cgqc_can_data._data_10[6] = RxMessage->Data[6];
	_cgqc_can_data._data_10[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_01(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_01[0] = RxMessage->Data[0];
	_dykj_can_data._data_01[1] = RxMessage->Data[1];
	_dykj_can_data._data_01[2] = RxMessage->Data[2];
	_dykj_can_data._data_01[3] = RxMessage->Data[3];
	_dykj_can_data._data_01[4] = RxMessage->Data[4];
	_dykj_can_data._data_01[5] = RxMessage->Data[5];
	_dykj_can_data._data_01[6] = RxMessage->Data[6];
	_dykj_can_data._data_01[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_02(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_02[0] = RxMessage->Data[0];
	_dykj_can_data._data_02[1] = RxMessage->Data[1];
	_dykj_can_data._data_02[2] = RxMessage->Data[2];
	_dykj_can_data._data_02[3] = RxMessage->Data[3];
	_dykj_can_data._data_02[4] = RxMessage->Data[4];
	_dykj_can_data._data_02[5] = RxMessage->Data[5];
	_dykj_can_data._data_02[6] = RxMessage->Data[6];
	_dykj_can_data._data_02[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_03(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_03[0] = RxMessage->Data[0];
	_dykj_can_data._data_03[1] = RxMessage->Data[1];
	_dykj_can_data._data_03[2] = RxMessage->Data[2];
	_dykj_can_data._data_03[3] = RxMessage->Data[3];
	_dykj_can_data._data_03[4] = RxMessage->Data[4];
	_dykj_can_data._data_03[5] = RxMessage->Data[5];
	_dykj_can_data._data_03[6] = RxMessage->Data[6];
	_dykj_can_data._data_03[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_04(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_04[0] = RxMessage->Data[0];
	_dykj_can_data._data_04[1] = RxMessage->Data[1];
	_dykj_can_data._data_04[2] = RxMessage->Data[2];
	_dykj_can_data._data_04[3] = RxMessage->Data[3];
	_dykj_can_data._data_04[4] = RxMessage->Data[4];
	_dykj_can_data._data_04[5] = RxMessage->Data[5];
	_dykj_can_data._data_04[6] = RxMessage->Data[6];
	_dykj_can_data._data_04[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_05(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_05[0] = RxMessage->Data[0];
	_dykj_can_data._data_05[1] = RxMessage->Data[1];
	_dykj_can_data._data_05[2] = RxMessage->Data[2];
	_dykj_can_data._data_05[3] = RxMessage->Data[3];
	_dykj_can_data._data_05[4] = RxMessage->Data[4];
	_dykj_can_data._data_05[5] = RxMessage->Data[5];
	_dykj_can_data._data_05[6] = RxMessage->Data[6];
	_dykj_can_data._data_05[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_06(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_06[0] = RxMessage->Data[0];
	_dykj_can_data._data_06[1] = RxMessage->Data[1];
	_dykj_can_data._data_06[2] = RxMessage->Data[2];
	_dykj_can_data._data_06[3] = RxMessage->Data[3];
	_dykj_can_data._data_06[4] = RxMessage->Data[4];
	_dykj_can_data._data_06[5] = RxMessage->Data[5];
	_dykj_can_data._data_06[6] = RxMessage->Data[6];
	_dykj_can_data._data_06[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_07(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_07[0] = RxMessage->Data[0];
	_dykj_can_data._data_07[1] = RxMessage->Data[1];
	_dykj_can_data._data_07[2] = RxMessage->Data[2];
	_dykj_can_data._data_07[3] = RxMessage->Data[3];
	_dykj_can_data._data_07[4] = RxMessage->Data[4];
	_dykj_can_data._data_07[5] = RxMessage->Data[5];
	_dykj_can_data._data_07[6] = RxMessage->Data[6];
	_dykj_can_data._data_07[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_08(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_08[0] = RxMessage->Data[0];
	_dykj_can_data._data_08[1] = RxMessage->Data[1];
	_dykj_can_data._data_08[2] = RxMessage->Data[2];
	_dykj_can_data._data_08[3] = RxMessage->Data[3];
	_dykj_can_data._data_08[4] = RxMessage->Data[4];
	_dykj_can_data._data_08[5] = RxMessage->Data[5];
	_dykj_can_data._data_08[6] = RxMessage->Data[6];
	_dykj_can_data._data_08[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_09(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_09[0] = RxMessage->Data[0];
	_dykj_can_data._data_09[1] = RxMessage->Data[1];
	_dykj_can_data._data_09[2] = RxMessage->Data[2];
	_dykj_can_data._data_09[3] = RxMessage->Data[3];
	_dykj_can_data._data_09[4] = RxMessage->Data[4];
	_dykj_can_data._data_09[5] = RxMessage->Data[5];
	_dykj_can_data._data_09[6] = RxMessage->Data[6];
	_dykj_can_data._data_09[7] = RxMessage->Data[7];
}

static _dykj_handle_msg_10(CanRxMsg *RxMessage)
{
	_dykj_can_data._data_10[0] = RxMessage->Data[0];
	_dykj_can_data._data_10[1] = RxMessage->Data[1];
	_dykj_can_data._data_10[2] = RxMessage->Data[2];
	_dykj_can_data._data_10[3] = RxMessage->Data[3];
	_dykj_can_data._data_10[4] = RxMessage->Data[4];
	_dykj_can_data._data_10[5] = RxMessage->Data[5];
	_dykj_can_data._data_10[6] = RxMessage->Data[6];
	_dykj_can_data._data_10[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_01(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_01[0] = RxMessage->Data[0];
	_jsyt_can_data._data_01[1] = RxMessage->Data[1];
	_jsyt_can_data._data_01[2] = RxMessage->Data[2];
	_jsyt_can_data._data_01[3] = RxMessage->Data[3];
	_jsyt_can_data._data_01[4] = RxMessage->Data[4];
	_jsyt_can_data._data_01[5] = RxMessage->Data[5];
	_jsyt_can_data._data_01[6] = RxMessage->Data[6];
	_jsyt_can_data._data_01[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_02(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_02[0] = RxMessage->Data[0];
	_jsyt_can_data._data_02[1] = RxMessage->Data[1];
	_jsyt_can_data._data_02[2] = RxMessage->Data[2];
	_jsyt_can_data._data_02[3] = RxMessage->Data[3];
	_jsyt_can_data._data_02[4] = RxMessage->Data[4];
	_jsyt_can_data._data_02[5] = RxMessage->Data[5];
	_jsyt_can_data._data_02[6] = RxMessage->Data[6];
	_jsyt_can_data._data_02[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_03(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_03[0] = RxMessage->Data[0];
	_jsyt_can_data._data_03[1] = RxMessage->Data[1];
	_jsyt_can_data._data_03[2] = RxMessage->Data[2];
	_jsyt_can_data._data_03[3] = RxMessage->Data[3];
	_jsyt_can_data._data_03[4] = RxMessage->Data[4];
	_jsyt_can_data._data_03[5] = RxMessage->Data[5];
	_jsyt_can_data._data_03[6] = RxMessage->Data[6];
	_jsyt_can_data._data_03[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_04(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_04[0] = RxMessage->Data[0];
	_jsyt_can_data._data_04[1] = RxMessage->Data[1];
	_jsyt_can_data._data_04[2] = RxMessage->Data[2];
	_jsyt_can_data._data_04[3] = RxMessage->Data[3];
	_jsyt_can_data._data_04[4] = RxMessage->Data[4];
	_jsyt_can_data._data_04[5] = RxMessage->Data[5];
	_jsyt_can_data._data_04[6] = RxMessage->Data[6];
	_jsyt_can_data._data_04[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_05(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_05[0] = RxMessage->Data[0];
	_jsyt_can_data._data_05[1] = RxMessage->Data[1];
	_jsyt_can_data._data_05[2] = RxMessage->Data[2];
	_jsyt_can_data._data_05[3] = RxMessage->Data[3];
	_jsyt_can_data._data_05[4] = RxMessage->Data[4];
	_jsyt_can_data._data_05[5] = RxMessage->Data[5];
	_jsyt_can_data._data_05[6] = RxMessage->Data[6];
	_jsyt_can_data._data_05[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_06(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_06[0] = RxMessage->Data[0];
	_jsyt_can_data._data_06[1] = RxMessage->Data[1];
	_jsyt_can_data._data_06[2] = RxMessage->Data[2];
	_jsyt_can_data._data_06[3] = RxMessage->Data[3];
	_jsyt_can_data._data_06[4] = RxMessage->Data[4];
	_jsyt_can_data._data_06[5] = RxMessage->Data[5];
	_jsyt_can_data._data_06[6] = RxMessage->Data[6];
	_jsyt_can_data._data_06[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_07(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_07[0] = RxMessage->Data[0];
	_jsyt_can_data._data_07[1] = RxMessage->Data[1];
	_jsyt_can_data._data_07[2] = RxMessage->Data[2];
	_jsyt_can_data._data_07[3] = RxMessage->Data[3];
	_jsyt_can_data._data_07[4] = RxMessage->Data[4];
	_jsyt_can_data._data_07[5] = RxMessage->Data[5];
	_jsyt_can_data._data_07[6] = RxMessage->Data[6];
	_jsyt_can_data._data_07[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_08(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_08[0] = RxMessage->Data[0];
	_jsyt_can_data._data_08[1] = RxMessage->Data[1];
	_jsyt_can_data._data_08[2] = RxMessage->Data[2];
	_jsyt_can_data._data_08[3] = RxMessage->Data[3];
	_jsyt_can_data._data_08[4] = RxMessage->Data[4];
	_jsyt_can_data._data_08[5] = RxMessage->Data[5];
	_jsyt_can_data._data_08[6] = RxMessage->Data[6];
	_jsyt_can_data._data_08[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_09(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_09[0] = RxMessage->Data[0];
	_jsyt_can_data._data_09[1] = RxMessage->Data[1];
	_jsyt_can_data._data_09[2] = RxMessage->Data[2];
	_jsyt_can_data._data_09[3] = RxMessage->Data[3];
	_jsyt_can_data._data_09[4] = RxMessage->Data[4];
	_jsyt_can_data._data_09[5] = RxMessage->Data[5];
	_jsyt_can_data._data_09[6] = RxMessage->Data[6];
	_jsyt_can_data._data_09[7] = RxMessage->Data[7];
}

static _jsyt_handle_msg_10(CanRxMsg *RxMessage)
{
	_jsyt_can_data._data_10[0] = RxMessage->Data[0];
	_jsyt_can_data._data_10[1] = RxMessage->Data[1];
	_jsyt_can_data._data_10[2] = RxMessage->Data[2];
	_jsyt_can_data._data_10[3] = RxMessage->Data[3];
	_jsyt_can_data._data_10[4] = RxMessage->Data[4];
	_jsyt_can_data._data_10[5] = RxMessage->Data[5];
	_jsyt_can_data._data_10[6] = RxMessage->Data[6];
	_jsyt_can_data._data_10[7] = RxMessage->Data[7];
}

static _vict_handle_msg_01(CanRxMsg *RxMessage)
{
	_vict_can_data._data_01[0] = RxMessage->Data[0];
	_vict_can_data._data_01[1] = RxMessage->Data[1];
	_vict_can_data._data_01[2] = RxMessage->Data[2];
	_vict_can_data._data_01[3] = RxMessage->Data[3];
	_vict_can_data._data_01[4] = RxMessage->Data[4];
	_vict_can_data._data_01[5] = RxMessage->Data[5];
	_vict_can_data._data_01[6] = RxMessage->Data[6];
	_vict_can_data._data_01[7] = RxMessage->Data[7];
}

static _vict_handle_msg_02(CanRxMsg *RxMessage)
{
	_vict_can_data._data_02[0] = RxMessage->Data[0];
	_vict_can_data._data_02[1] = RxMessage->Data[1];
	_vict_can_data._data_02[2] = RxMessage->Data[2];
	_vict_can_data._data_02[3] = RxMessage->Data[3];
	_vict_can_data._data_02[4] = RxMessage->Data[4];
	_vict_can_data._data_02[5] = RxMessage->Data[5];
	_vict_can_data._data_02[6] = RxMessage->Data[6];
	_vict_can_data._data_02[7] = RxMessage->Data[7];
}

static _vict_handle_msg_03(CanRxMsg *RxMessage)
{
	_vict_can_data._data_03[0] = RxMessage->Data[0];
	_vict_can_data._data_03[1] = RxMessage->Data[1];
	_vict_can_data._data_03[2] = RxMessage->Data[2];
	_vict_can_data._data_03[3] = RxMessage->Data[3];
	_vict_can_data._data_03[4] = RxMessage->Data[4];
	_vict_can_data._data_03[5] = RxMessage->Data[5];
	_vict_can_data._data_03[6] = RxMessage->Data[6];
	_vict_can_data._data_03[7] = RxMessage->Data[7];
}

static _vict_handle_msg_04(CanRxMsg *RxMessage)
{
	_vict_can_data._data_04[0] = RxMessage->Data[0];
	_vict_can_data._data_04[1] = RxMessage->Data[1];
	_vict_can_data._data_04[2] = RxMessage->Data[2];
	_vict_can_data._data_04[3] = RxMessage->Data[3];
	_vict_can_data._data_04[4] = RxMessage->Data[4];
	_vict_can_data._data_04[5] = RxMessage->Data[5];
	_vict_can_data._data_04[6] = RxMessage->Data[6];
	_vict_can_data._data_04[7] = RxMessage->Data[7];
}

static _vict_handle_msg_05(CanRxMsg *RxMessage)
{
	_vict_can_data._data_05[0] = RxMessage->Data[0];
	_vict_can_data._data_05[1] = RxMessage->Data[1];
	_vict_can_data._data_05[2] = RxMessage->Data[2];
	_vict_can_data._data_05[3] = RxMessage->Data[3];
	_vict_can_data._data_05[4] = RxMessage->Data[4];
	_vict_can_data._data_05[5] = RxMessage->Data[5];
	_vict_can_data._data_05[6] = RxMessage->Data[6];
	_vict_can_data._data_05[7] = RxMessage->Data[7];
}

static _vict_handle_msg_06(CanRxMsg *RxMessage)
{
	_vict_can_data._data_06[0] = RxMessage->Data[0];
	_vict_can_data._data_06[1] = RxMessage->Data[1];
	_vict_can_data._data_06[2] = RxMessage->Data[2];
	_vict_can_data._data_06[3] = RxMessage->Data[3];
	_vict_can_data._data_06[4] = RxMessage->Data[4];
	_vict_can_data._data_06[5] = RxMessage->Data[5];
	_vict_can_data._data_06[6] = RxMessage->Data[6];
	_vict_can_data._data_06[7] = RxMessage->Data[7];
}

static _vict_handle_msg_07(CanRxMsg *RxMessage)
{
	_vict_can_data._data_07[0] = RxMessage->Data[0];
	_vict_can_data._data_07[1] = RxMessage->Data[1];
	_vict_can_data._data_07[2] = RxMessage->Data[2];
	_vict_can_data._data_07[3] = RxMessage->Data[3];
	_vict_can_data._data_07[4] = RxMessage->Data[4];
	_vict_can_data._data_07[5] = RxMessage->Data[5];
	_vict_can_data._data_07[6] = RxMessage->Data[6];
	_vict_can_data._data_07[7] = RxMessage->Data[7];
}

static _vict_handle_msg_08(CanRxMsg *RxMessage)
{
	_vict_can_data._data_08[0] = RxMessage->Data[0];
	_vict_can_data._data_08[1] = RxMessage->Data[1];
	_vict_can_data._data_08[2] = RxMessage->Data[2];
	_vict_can_data._data_08[3] = RxMessage->Data[3];
	_vict_can_data._data_08[4] = RxMessage->Data[4];
	_vict_can_data._data_08[5] = RxMessage->Data[5];
	_vict_can_data._data_08[6] = RxMessage->Data[6];
	_vict_can_data._data_08[7] = RxMessage->Data[7];
}

static _vict_handle_msg_09(CanRxMsg *RxMessage)
{
	_vict_can_data._data_09[0] = RxMessage->Data[0];
	_vict_can_data._data_09[1] = RxMessage->Data[1];
	_vict_can_data._data_09[2] = RxMessage->Data[2];
	_vict_can_data._data_09[3] = RxMessage->Data[3];
	_vict_can_data._data_09[4] = RxMessage->Data[4];
	_vict_can_data._data_09[5] = RxMessage->Data[5];
	_vict_can_data._data_09[6] = RxMessage->Data[6];
	_vict_can_data._data_09[7] = RxMessage->Data[7];
}

static _vict_handle_msg_10(CanRxMsg *RxMessage)
{
	_vict_can_data._data_10[0] = RxMessage->Data[0];
	_vict_can_data._data_10[1] = RxMessage->Data[1];
	_vict_can_data._data_10[2] = RxMessage->Data[2];
	_vict_can_data._data_10[3] = RxMessage->Data[3];
	_vict_can_data._data_10[4] = RxMessage->Data[4];
	_vict_can_data._data_10[5] = RxMessage->Data[5];
	_vict_can_data._data_10[6] = RxMessage->Data[6];
	_vict_can_data._data_10[7] = RxMessage->Data[7];
}

static _vict_handle_msg_11(CanRxMsg *RxMessage)
{
	_vict_can_data._data_11[0] = RxMessage->Data[0];
	_vict_can_data._data_11[1] = RxMessage->Data[1];
	_vict_can_data._data_11[2] = RxMessage->Data[2];
	_vict_can_data._data_11[3] = RxMessage->Data[3];
	_vict_can_data._data_11[4] = RxMessage->Data[4];
	_vict_can_data._data_11[5] = RxMessage->Data[5];
	_vict_can_data._data_11[6] = RxMessage->Data[6];
	_vict_can_data._data_11[7] = RxMessage->Data[7];
}

static _vict_handle_msg_12(CanRxMsg *RxMessage)
{
	_vict_can_data._data_12[0] = RxMessage->Data[0];
	_vict_can_data._data_12[1] = RxMessage->Data[1];
	_vict_can_data._data_12[2] = RxMessage->Data[2];
	_vict_can_data._data_12[3] = RxMessage->Data[3];
	_vict_can_data._data_12[4] = RxMessage->Data[4];
	_vict_can_data._data_12[5] = RxMessage->Data[5];
	_vict_can_data._data_12[6] = RxMessage->Data[6];
	_vict_can_data._data_12[7] = RxMessage->Data[7];
}

static _vict_handle_msg_13(CanRxMsg *RxMessage)
{
	_vict_can_data._data_13[0] = RxMessage->Data[0];
	_vict_can_data._data_13[1] = RxMessage->Data[1];
	_vict_can_data._data_13[2] = RxMessage->Data[2];
	_vict_can_data._data_13[3] = RxMessage->Data[3];
	_vict_can_data._data_13[4] = RxMessage->Data[4];
	_vict_can_data._data_13[5] = RxMessage->Data[5];
	_vict_can_data._data_13[6] = RxMessage->Data[6];
	_vict_can_data._data_13[7] = RxMessage->Data[7];
}

static _vict_handle_msg_14(CanRxMsg *RxMessage)
{
	_vict_can_data._data_14[0] = RxMessage->Data[0];
	_vict_can_data._data_14[1] = RxMessage->Data[1];
	_vict_can_data._data_14[2] = RxMessage->Data[2];
	_vict_can_data._data_14[3] = RxMessage->Data[3];
	_vict_can_data._data_14[4] = RxMessage->Data[4];
	_vict_can_data._data_14[5] = RxMessage->Data[5];
	_vict_can_data._data_14[6] = RxMessage->Data[6];
	_vict_can_data._data_14[7] = RxMessage->Data[7];
}

static _vict_handle_msg_15(CanRxMsg *RxMessage)
{
	_vict_can_data._data_15[0] = RxMessage->Data[0];
	_vict_can_data._data_15[1] = RxMessage->Data[1];
	_vict_can_data._data_15[2] = RxMessage->Data[2];
	_vict_can_data._data_15[3] = RxMessage->Data[3];
	_vict_can_data._data_15[4] = RxMessage->Data[4];
	_vict_can_data._data_15[5] = RxMessage->Data[5];
	_vict_can_data._data_15[6] = RxMessage->Data[6];
	_vict_can_data._data_15[7] = RxMessage->Data[7];
}

static _vict_handle_msg_16(CanRxMsg *RxMessage)
{
	_vict_can_data._data_16[0] = RxMessage->Data[0];
	_vict_can_data._data_16[1] = RxMessage->Data[1];
	_vict_can_data._data_16[2] = RxMessage->Data[2];
	_vict_can_data._data_16[3] = RxMessage->Data[3];
	_vict_can_data._data_16[4] = RxMessage->Data[4];
	_vict_can_data._data_16[5] = RxMessage->Data[5];
	_vict_can_data._data_16[6] = RxMessage->Data[6];
	_vict_can_data._data_16[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_01(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_01[0] = RxMessage->Data[0];
	_ydzy_can_data._data_01[1] = RxMessage->Data[1];
	_ydzy_can_data._data_01[2] = RxMessage->Data[2];
	_ydzy_can_data._data_01[3] = RxMessage->Data[3];
	_ydzy_can_data._data_01[4] = RxMessage->Data[4];
	_ydzy_can_data._data_01[5] = RxMessage->Data[5];
	_ydzy_can_data._data_01[6] = RxMessage->Data[6];
	_ydzy_can_data._data_01[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_02(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_02[0] = RxMessage->Data[0];
	_ydzy_can_data._data_02[1] = RxMessage->Data[1];
	_ydzy_can_data._data_02[2] = RxMessage->Data[2];
	_ydzy_can_data._data_02[3] = RxMessage->Data[3];
	_ydzy_can_data._data_02[4] = RxMessage->Data[4];
	_ydzy_can_data._data_02[5] = RxMessage->Data[5];
	_ydzy_can_data._data_02[6] = RxMessage->Data[6];
	_ydzy_can_data._data_02[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_03(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_03[0] = RxMessage->Data[0];
	_ydzy_can_data._data_03[1] = RxMessage->Data[1];
	_ydzy_can_data._data_03[2] = RxMessage->Data[2];
	_ydzy_can_data._data_03[3] = RxMessage->Data[3];
	_ydzy_can_data._data_03[4] = RxMessage->Data[4];
	_ydzy_can_data._data_03[5] = RxMessage->Data[5];
	_ydzy_can_data._data_03[6] = RxMessage->Data[6];
	_ydzy_can_data._data_03[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_04(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_04[0] = RxMessage->Data[0];
	_ydzy_can_data._data_04[1] = RxMessage->Data[1];
	_ydzy_can_data._data_04[2] = RxMessage->Data[2];
	_ydzy_can_data._data_04[3] = RxMessage->Data[3];
	_ydzy_can_data._data_04[4] = RxMessage->Data[4];
	_ydzy_can_data._data_04[5] = RxMessage->Data[5];
	_ydzy_can_data._data_04[6] = RxMessage->Data[6];
	_ydzy_can_data._data_04[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_05(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_05[0] = RxMessage->Data[0];
	_ydzy_can_data._data_05[1] = RxMessage->Data[1];
	_ydzy_can_data._data_05[2] = RxMessage->Data[2];
	_ydzy_can_data._data_05[3] = RxMessage->Data[3];
	_ydzy_can_data._data_05[4] = RxMessage->Data[4];
	_ydzy_can_data._data_05[5] = RxMessage->Data[5];
	_ydzy_can_data._data_05[6] = RxMessage->Data[6];
	_ydzy_can_data._data_05[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_06(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_06[0] = RxMessage->Data[0];
	_ydzy_can_data._data_06[1] = RxMessage->Data[1];
	_ydzy_can_data._data_06[2] = RxMessage->Data[2];
	_ydzy_can_data._data_06[3] = RxMessage->Data[3];
	_ydzy_can_data._data_06[4] = RxMessage->Data[4];
	_ydzy_can_data._data_06[5] = RxMessage->Data[5];
	_ydzy_can_data._data_06[6] = RxMessage->Data[6];
	_ydzy_can_data._data_06[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_07(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_07[0] = RxMessage->Data[0];
	_ydzy_can_data._data_07[1] = RxMessage->Data[1];
	_ydzy_can_data._data_07[2] = RxMessage->Data[2];
	_ydzy_can_data._data_07[3] = RxMessage->Data[3];
	_ydzy_can_data._data_07[4] = RxMessage->Data[4];
	_ydzy_can_data._data_07[5] = RxMessage->Data[5];
	_ydzy_can_data._data_07[6] = RxMessage->Data[6];
	_ydzy_can_data._data_07[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_08(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_08[0] = RxMessage->Data[0];
	_ydzy_can_data._data_08[1] = RxMessage->Data[1];
	_ydzy_can_data._data_08[2] = RxMessage->Data[2];
	_ydzy_can_data._data_08[3] = RxMessage->Data[3];
	_ydzy_can_data._data_08[4] = RxMessage->Data[4];
	_ydzy_can_data._data_08[5] = RxMessage->Data[5];
	_ydzy_can_data._data_08[6] = RxMessage->Data[6];
	_ydzy_can_data._data_08[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_09(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_09[0] = RxMessage->Data[0];
	_ydzy_can_data._data_09[1] = RxMessage->Data[1];
	_ydzy_can_data._data_09[2] = RxMessage->Data[2];
	_ydzy_can_data._data_09[3] = RxMessage->Data[3];
	_ydzy_can_data._data_09[4] = RxMessage->Data[4];
	_ydzy_can_data._data_09[5] = RxMessage->Data[5];
	_ydzy_can_data._data_09[6] = RxMessage->Data[6];
	_ydzy_can_data._data_09[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_10(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_10[0] = RxMessage->Data[0];
	_ydzy_can_data._data_10[1] = RxMessage->Data[1];
	_ydzy_can_data._data_10[2] = RxMessage->Data[2];
	_ydzy_can_data._data_10[3] = RxMessage->Data[3];
	_ydzy_can_data._data_10[4] = RxMessage->Data[4];
	_ydzy_can_data._data_10[5] = RxMessage->Data[5];
	_ydzy_can_data._data_10[6] = RxMessage->Data[6];
	_ydzy_can_data._data_10[7] = RxMessage->Data[7];
}

static _ydzy_handle_msg_11(CanRxMsg *RxMessage)
{
	_ydzy_can_data._data_11[0] = RxMessage->Data[0];
	_ydzy_can_data._data_11[1] = RxMessage->Data[1];
	_ydzy_can_data._data_11[2] = RxMessage->Data[2];
	_ydzy_can_data._data_11[3] = RxMessage->Data[3];
	_ydzy_can_data._data_11[4] = RxMessage->Data[4];
	_ydzy_can_data._data_11[5] = RxMessage->Data[5];
	_ydzy_can_data._data_11[6] = RxMessage->Data[6];
	_ydzy_can_data._data_11[7] = RxMessage->Data[7];
}


void can_handle_receive(CanRxMsg *RxMessage)
{
	switch(RxMessage->ExtId)
	{
		case CAN_FILTER_ID_CAR_VIN:
			_handle_msg_car_vin(RxMessage);
			break;
			
		case CAN_FILTER_ID_MILEAGE:
			_handle_msg_mileage(RxMessage);
			break;
			
		case CAN_FILTER_ID_MILEAGE_2:
			_handle_msg_mileage2(RxMessage);
			break;
			
		case CAN_FILTER_ID_MOTOR_SPEED:
			_handle_msg_motor_speed(RxMessage);
			break;
			
		case CAN_FILTER_ID_BATTERY:
			_handle_msg_battery(RxMessage);
			break;
			
		case CAN_FILTER_ID_CAR_STATUS:
			_handle_msg_car_status(RxMessage);
			break;
			
		case CAN_FILTER_ID_MOTOR_TORQUE:
			_handle_msg_motor_torque(RxMessage);
			break;
			
		case CAN_FILTER_ID_MOTOR_TEMP:
			_handle_msg_motor_temp(RxMessage);
			break;
			
		case CAN_FILTER_ID_MOTOR_CURRENT:
			_handle_msg_motor_current(RxMessage);
			break;
			
		case CAN_FILTER_ID_CHARGE_VOLTAGE:
			_handle_msg_charge_voltage(RxMessage);
			_ax_handle_msg_16(RxMessage);
			_xx_handle_msg_11(RxMessage);
		  _dykj_handle_msg_10(RxMessage);
			break;
			
		case CAN_FILTER_ID_RESISTANCE:
			_handle_msg_resistance(RxMessage);
			break;
			
		case CAN_FILTER_ID_ERROR_ALARM:
			_handle_msg_error_alarm(RxMessage);
			break;

		//������AX����Э��
		case CAN_AX_FILTER_ID_TOTAL_CAR_CONTROL:
			_ax_handle_msg_01(RxMessage);
			break;

		case CAN_AX_FILTER_ID_TOTAL_BATTERY_CONTROL:
			_ax_handle_msg_02(RxMessage);
			break;

		case CAN_AX_FILTER_ID_DISPLAY_METER_1:
			_ax_handle_msg_03(RxMessage);
			break;

		case CAN_AX_FILTER_ID_DISPLAY_METER_2:
			_ax_handle_msg_04(RxMessage);
			break;

		case CAN_AX_FILTER_ID_REGISTER_INFO_1:
			_ax_handle_msg_05(RxMessage);
			break;

		case CAN_AX_FILTER_ID_REGISTER_INFO_2:
			_ax_handle_msg_06(RxMessage);
			break;

		case CAN_AX_FILTER_ID_BATTERY_INFO_1:
			_ax_handle_msg_09(RxMessage);
			break;

		case CAN_AX_FILTER_ID_BATTERY_INFO_2:
			_ax_handle_msg_10(RxMessage);
			break;

		case CAN_AX_FILTER_ID_BATTERY_INFO_3:
			_ax_handle_msg_11(RxMessage);
			break;

		case CAN_AX_FILTER_ID_BATTERY_ALARM_INFO_1:
			_ax_handle_msg_12(RxMessage);
			break;

		case CAN_AX_FILTER_ID_BATTERY_ALARM_INFO_2:
			_ax_handle_msg_13(RxMessage);
			break;

		case CAN_AX_FILTER_ID_MOTOR_1:
			_ax_handle_msg_14(RxMessage);
			break;

		case CAN_AX_FILTER_ID_MOTOR_2:
			_ax_handle_msg_15(RxMessage);
			break;

		case CAN_XX_FILTER_ID_01:
			_xx_handle_msg_01(RxMessage);
			_dykj_handle_msg_02(RxMessage);
			break;
		case CAN_XX_FILTER_ID_02:
			_xx_handle_msg_02(RxMessage);
			_dykj_handle_msg_03(RxMessage);
			_jsyt_handle_msg_03(RxMessage);
			break;
		case CAN_XX_FILTER_ID_03:
			_xx_handle_msg_03(RxMessage);
			_dykj_handle_msg_04(RxMessage);
		  _jsyt_handle_msg_04(RxMessage);
			break;
		case CAN_XX_FILTER_ID_04:
			_xx_handle_msg_04(RxMessage);
			_dykj_handle_msg_05(RxMessage);
			break;
		case CAN_XX_FILTER_ID_05:
			_xx_handle_msg_05(RxMessage);
		  _dykj_handle_msg_06(RxMessage);
		  _jsyt_handle_msg_06(RxMessage);
			break;
		case CAN_XX_FILTER_ID_06:
			_xx_handle_msg_06(RxMessage);
			_dykj_handle_msg_07(RxMessage);
		  _jsyt_handle_msg_07(RxMessage);
			break;
		case CAN_XX_FILTER_ID_07:
			_xx_handle_msg_07(RxMessage);
			break;
		case CAN_XX_FILTER_ID_08:
			_xx_handle_msg_08(RxMessage);
			break;
		case CAN_XX_FILTER_ID_09:
			_xx_handle_msg_09(RxMessage);
		  _dykj_handle_msg_08(RxMessage);
		  _jsyt_handle_msg_08(RxMessage);
			break;
		case CAN_XX_FILTER_ID_10:
			_xx_handle_msg_10(RxMessage);
		  _dykj_handle_msg_09(RxMessage);
		  _jsyt_handle_msg_09(RxMessage);
			break;
		/*
		case CAN_XX_FILTER_ID_11:
			break;
		*/
		case CAN_XX_FILTER_ID_12:
			_xx_handle_msg_12(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_01:
			_cgqc_handle_msg_01(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_02:
			_cgqc_handle_msg_02(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_03:
			_cgqc_handle_msg_03(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_04:
			_cgqc_handle_msg_04(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_05:
			_cgqc_handle_msg_05(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_06:
			_cgqc_handle_msg_06(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_07:
			_cgqc_handle_msg_07(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_08:
			_cgqc_handle_msg_08(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_09:
			_cgqc_handle_msg_09(RxMessage);
			break;

		case CAN_CGQC_FILTER_ID_10:
			_cgqc_handle_msg_10(RxMessage);
			break;

		case CAN_DYKJ_FILTER_ID_01:
			_dykj_handle_msg_01(RxMessage);
			break;
		/*
		case CAN_DYKJ_FILTER_ID_02:
			_dykj_handle_msg_02(RxMessage);
			break;
		
		case CAN_DYKJ_FILTER_ID_03:
			_dykj_handle_msg_03(RxMessage);
			break;
		
		case CAN_DYKJ_FILTER_ID_04:
			_dykj_handle_msg_04(RxMessage);
			break;
		
		case CAN_DYKJ_FILTER_ID_05:
			_dykj_handle_msg_05(RxMessage);
			break;
		
		case CAN_DYKJ_FILTER_ID_06:
			_dykj_handle_msg_06(RxMessage);
			break;
		
		case CAN_DYKJ_FILTER_ID_07:
			_dykj_handle_msg_07(RxMessage);
			break;
		
		case CAN_DYKJ_FILTER_ID_08:
			_dykj_handle_msg_08(RxMessage);
			break;
		
		case CAN_DYKJ_FILTER_ID_09:
			_dykj_handle_msg_09(RxMessage);
			break;
			
		case CAN_DYKJ_FILTER_ID_10:
			_dykj_handle_msg_10(RxMessage);
			break;
		*/	

		case CAN_JSYT_FILTER_ID_01:
			_jsyt_handle_msg_01(RxMessage);
			break;
		case CAN_JSYT_FILTER_ID_02:
			_jsyt_handle_msg_02(RxMessage);
			break;
		/*
		case CAN_JSYT_FILTER_ID_03:
			_jsyt_handle_msg_03(RxMessage);
			break;
		
		case CAN_JSYT_FILTER_ID_04:
			_jsyt_handle_msg_04(RxMessage);
			break;
		*/
		case CAN_JSYT_FILTER_ID_05:
			_jsyt_handle_msg_05(RxMessage);
			break;
		/*
		case CAN_JSYT_FILTER_ID_06:
			_jsyt_handle_msg_06(RxMessage);
			break;
		
		case CAN_JSYT_FILTER_ID_07:
			_jsyt_handle_msg_07(RxMessage);
			break;
		
		case CAN_JSYT_FILTER_ID_08:
			_jsyt_handle_msg_08(RxMessage);
			break;
		
		case CAN_JSYT_FILTER_ID_09:
			_jsyt_handle_msg_09(RxMessage);
			break;
		*/
		case CAN_JSYT_FILTER_ID_10:
			_jsyt_handle_msg_10(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_01:
			_vict_handle_msg_01(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_02:
			_vict_handle_msg_02(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_03:
			_vict_handle_msg_03(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_04:
			_vict_handle_msg_04(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_05:
			_vict_handle_msg_05(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_06:
			_vict_handle_msg_06(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_07:
			_vict_handle_msg_07(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_08:
			_vict_handle_msg_08(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_09:
			_vict_handle_msg_09(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_10:
			_vict_handle_msg_10(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_11:
			_vict_handle_msg_11(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_12:
			_vict_handle_msg_12(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_13:
			_vict_handle_msg_13(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_14:
			_vict_handle_msg_14(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_15:
			_vict_handle_msg_15(RxMessage);
			break;

		case CAN_VICT_FILTER_ID_16:
			_vict_handle_msg_16(RxMessage);
			break;
			
		case CAN_YDZY_FILTER_ID_01:
			_ydzy_handle_msg_01(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_02:
			_ydzy_handle_msg_02(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_03:
			_ydzy_handle_msg_03(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_04:
			_ydzy_handle_msg_04(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_05:
			_ydzy_handle_msg_05(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_06:
			_ydzy_handle_msg_06(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_07:
			_ydzy_handle_msg_07(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_08:
			_ydzy_handle_msg_08(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_09:
			_ydzy_handle_msg_09(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_10:
			_ydzy_handle_msg_10(RxMessage);
			break;

		case CAN_YDZY_FILTER_ID_11:
			_ydzy_handle_msg_11(RxMessage);
			break;
		
		default:
			break;
	}
}

#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�			    
void USB_LP_CAN1_RX0_IRQHandler(void)//�жϷ�����
{
	CanRxMsg RxMessage;
  memset(&RxMessage, 0, sizeof(RxMessage));
  CAN_Receive(CAN1, 0, &RxMessage);
	CLI();
	can_handle_receive(&RxMessage);
	SEI();
}
#endif
