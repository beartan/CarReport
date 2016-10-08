
#ifndef __CAN_H
#define __CAN_H

#include "Dev.h"

//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	1		//0,不使能;1,使能.	

#define CAN_FILTER_ID_CAR_VIN         0x18011801//01.车辆VIN
#define CAN_FILTER_ID_MILEAGE         0x18030703//02.里程数/续航里程
#define CAN_FILTER_ID_MILEAGE_2				0x18010B01//02-2续航里程(覆盖)
#define CAN_FILTER_ID_MOTOR_SPEED     0x18010801//03.电池电压/电池电流/车速/电机转速	
#define CAN_FILTER_ID_BATTERY         0x18011B01//04.单体电池最高/最低电压
#define CAN_FILTER_ID_CAR_STATUS      0x18010901//05.SOC/车辆状态/充电起止时间/车辆状态/启动/工作状态信号
#define CAN_FILTER_ID_MOTOR_TORQUE    0x18F11F05//06.电机转矩
#define CAN_FILTER_ID_MOTOR_TEMP      0x18F18D05//07.驱动电机温度/逆变器温度
#define CAN_FILTER_ID_MOTOR_CURRENT   0x18F12005//08.电机电流/电机电压
#define CAN_FILTER_ID_CHARGE_VOLTAGE  0x18FF50E5//09.充电电压/充电电流
#define CAN_FILTER_ID_RESISTANCE      0x18011901//10.绝缘电阻
#define CAN_FILTER_ID_ERROR_ALARM     0x18010A01//11.故障报警


#define CAN_AX_FILTER_ID_CAR_VIN               0x18011801//01.车辆VIN
#define CAN_AX_FILTER_ID_TOTAL_CAR_CONTROL     0x0C06D128//02.仪表to整车控制器
#define CAN_AX_FILTER_ID_TOTAL_BATTERY_CONTROL  0x0C06F3D0//03.整车控制器to电池管理系统
#define CAN_AX_FILTER_ID_DISPLAY_METER_1		   0x0C0628F3//电池管理系统to显示仪表
#define CAN_AX_FILTER_ID_DISPLAY_METER_2		   0x0C0428F3//电池管理系统to显示仪表
#define CAN_AX_FILTER_ID_REGISTER_INFO_1       0x0C10D0F3//BMS注册信息1 ID=0x0C10D0F3
#define CAN_AX_FILTER_ID_REGISTER_INFO_2       0x0C11D0F3//BMS注册信息2 ID=0x0C11D0F3
#define CAN_AX_FILTER_ID_BATTERY_INFO_1        0x0C20D0F3//BMS电池实时信息1   ID=0x0C20D0F3
#define CAN_AX_FILTER_ID_BATTERY_INFO_2        0x0C21D0F3//BMS电池实时信息2   ID=0x0C21D0F3
#define CAN_AX_FILTER_ID_BATTERY_INFO_3        0x0C22D0F3//BMS电池实时信息3   ID=0x0C22D0F3
#define CAN_AX_FILTER_ID_BATTERY_ALARM_INFO_1  0x0C30D0F3//BMS电池报警信息1  ID=0x0C30D0F3
#define CAN_AX_FILTER_ID_BATTERY_ALARM_INFO_2  0x0C31D0F3//BMS电池报警信息2  ID=0x0C31D0F3
#define CAN_AX_FILTER_ID_MOTOR_1 							 0x10088A9E// ID=0x10088A9E
#define CAN_AX_FILTER_ID_MOTOR_2 					     0x10098A9E// ID=0x10098A9E
//#define CAN_AX_FILTER_ID_CHARGE                0x18FF50E5


#define CAN_XX_FILTER_ID_01 0x18FEF1A6
#define CAN_XX_FILTER_ID_02 0x18FEF2A6
#define CAN_XX_FILTER_ID_03 0x18FEF3A6
#define CAN_XX_FILTER_ID_04 0x18FEF4A6
#define CAN_XX_FILTER_ID_05 0x1000A6A9
#define CAN_XX_FILTER_ID_06 0x1D00A6A9
#define CAN_XX_FILTER_ID_07 0x18F15AF3
#define CAN_XX_FILTER_ID_08 0x0C03D0E7
#define CAN_XX_FILTER_ID_09 0x0800A6A9
#define CAN_XX_FILTER_ID_10 0x1C00A6A9
//#define CAN_XX_FILTER_ID_11 0x18FF50E5
#define CAN_XX_FILTER_ID_12 0x183C4A49



#define CAN_CGQC_FILTER_ID_01 0x0CF00606
#define CAN_CGQC_FILTER_ID_02 0x18F01A06
#define CAN_CGQC_FILTER_ID_03 0x18F03406
#define CAN_CGQC_FILTER_ID_04 0x0CF50124
#define CAN_CGQC_FILTER_ID_05 0x18F51124
#define CAN_CGQC_FILTER_ID_06 0x18F52224
#define CAN_CGQC_FILTER_ID_07 0x18F53324
#define CAN_CGQC_FILTER_ID_08 0x18FD7524
#define CAN_CGQC_FILTER_ID_09 0x18EB2440
#define CAN_CGQC_FILTER_ID_10 0x18F43917
#define CAN_CGQC_FILTER_ID_11 
#define CAN_CGQC_FILTER_ID_12 




#define CAN_DYKJ_FILTER_ID_01 0x18FEF0A6
#define CAN_DYKJ_FILTER_ID_02 0x18FEF1A6
#define CAN_DYKJ_FILTER_ID_03 0x18FEF2A6
#define CAN_DYKJ_FILTER_ID_04 0x18FEF3A6
#define CAN_DYKJ_FILTER_ID_05 0x18FEF4A6
#define CAN_DYKJ_FILTER_ID_06 0x1000A6A9
#define CAN_DYKJ_FILTER_ID_07 0x1D00A6A9
#define CAN_DYKJ_FILTER_ID_08 0x0800A6A9
#define CAN_DYKJ_FILTER_ID_09 0x1C00A6A9
#define CAN_DYKJ_FILTER_ID_10 0x18FF50E5
#define CAN_DYKJ_FILTER_ID_11 
#define CAN_DYKJ_FILTER_ID_12 



#define CAN_JSYT_FILTER_ID_01 0x18E4A5A6
#define CAN_JSYT_FILTER_ID_02 0x0C09A6A7
#define CAN_JSYT_FILTER_ID_03 0x18FEF2A6
#define CAN_JSYT_FILTER_ID_04 0x18FEF3A6
#define CAN_JSYT_FILTER_ID_05 0x0901A6A7
#define CAN_JSYT_FILTER_ID_06 0x1000A6A9
#define CAN_JSYT_FILTER_ID_07 0x1D00A6A9
#define CAN_JSYT_FILTER_ID_08 0x0800A6A9
#define CAN_JSYT_FILTER_ID_09 0x1C00A6A9
#define CAN_JSYT_FILTER_ID_10 0x180050E5
#define CAN_JSYT_FILTER_ID_11 
#define CAN_JSYT_FILTER_ID_12 


#define CAN_VICT_FILTER_ID_01 0xCF100D0
#define CAN_VICT_FILTER_ID_02 0xCF101D0
#define CAN_VICT_FILTER_ID_03 0xCF10BD0
#define CAN_VICT_FILTER_ID_04 0x18F43A17
#define CAN_VICT_FILTER_ID_05 0xCFF0008
#define CAN_VICT_FILTER_ID_06 0xCFF0108
#define CAN_VICT_FILTER_ID_07 0xCFF0208
#define CAN_VICT_FILTER_ID_08 0x18F212F3
#define CAN_VICT_FILTER_ID_09 0x18F213F3
#define CAN_VICT_FILTER_ID_10 0x18F214F3
#define CAN_VICT_FILTER_ID_11 0x181E17F3
#define CAN_VICT_FILTER_ID_12 0x181F17F3
#define CAN_VICT_FILTER_ID_13 0x182017F3
#define CAN_VICT_FILTER_ID_14 0x18A017F3
#define CAN_VICT_FILTER_ID_15 0x0CF217F3
#define CAN_VICT_FILTER_ID_16 0x18F302E4
#define CAN_VICT_FILTER_ID_17 



#define CAN_YDZY_FILTER_ID_01 0x18FF01F1
#define CAN_YDZY_FILTER_ID_02 0x18FF02F1
#define CAN_YDZY_FILTER_ID_03 0x18FF03F1
#define CAN_YDZY_FILTER_ID_04 0x18FF04F1
#define CAN_YDZY_FILTER_ID_05 0x18FF05F1
#define CAN_YDZY_FILTER_ID_06 0x18FF06F1
#define CAN_YDZY_FILTER_ID_07 0x18FF07F1
#define CAN_YDZY_FILTER_ID_08 0xC07F0A7
#define CAN_YDZY_FILTER_ID_09 0xC08A7F0
#define CAN_YDZY_FILTER_ID_10 0xC09A7F0
#define CAN_YDZY_FILTER_ID_11 0xC0AA7F0
#define CAN_YDZY_FILTER_ID_12 



extern uint8_t _car_vin_tmp[17];
extern uint8_t _car_vin[17];//采用两份数据，保证数据同步


typedef struct
{
	uint8_t _mileage[8];         
	uint8_t _motor_speed[8];			
	uint8_t _battery[8];					
	uint8_t _car_status[8];				
	uint8_t _motor_torque[8];			
	uint8_t _motor_temp[8];				
	uint8_t _motor_current[8];		
	uint8_t _charge_voltage[8];		
	uint8_t _resistance[8];				
	uint8_t _error_alarm[8];			
}hy_can_data_t;

typedef struct
{
	uint8_t _data_01[7];         
	uint8_t _data_02[4];			
	uint8_t _data_03[8];					
	uint8_t _data_04[8];				
	uint8_t _data_05[1];			
	uint8_t _data_06[8];						
	uint8_t _data_09[4];				
	uint8_t _data_10[8];
	uint8_t _data_11[8];
	uint8_t _data_12[5];
	uint8_t _data_13[4];
	uint8_t _data_14[8];
	uint8_t _data_15[6];
	uint8_t _data_16[5];
}ax_can_data_t;

typedef struct
{
	uint8_t _data_01[7];//0x18FEF1A6        
	uint8_t _data_02[7];//0x18FEF2A6			
	uint8_t _data_03[6];//0x18FEF3A6						
	uint8_t _data_04[7];//0x18FEF4A6				
	uint8_t _data_05[7];//0x1000A6A9			
	uint8_t _data_06[2];//0x1D00A6A9						
	uint8_t _data_07[2];//0x18F15AF3				
	uint8_t _data_08[7];//0x0C03D0E7
	uint8_t _data_09[6];//0x0800A6A9	
	uint8_t _data_10[4];//0x1C00A6A9
	uint8_t _data_11[4];//0x18FF50E5
	uint8_t _data_12[2];//0x183C4A49
}xx_can_data_t;

typedef struct
{
	uint8_t _data_01[8];//0x0CF00606 
	uint8_t _data_02[8];//0x18F01A06
	uint8_t _data_03[8];//0x18F03406
	uint8_t _data_04[8];//0x0CF50124
	uint8_t _data_05[8];//0x18F51124
	uint8_t _data_06[8];//0x18F52224
	uint8_t _data_07[8];//0x18F53324
	uint8_t _data_08[8];//0x18FD7524
	uint8_t _data_09[8];//0x18EB2440
	uint8_t _data_10[8];//0x18F43917
	uint8_t _data_11[8];
	uint8_t _data_12[8];
}cgqc_can_data_t;


typedef struct
{
	uint8_t _data_01[8];//0x18FEF0A6 
	uint8_t _data_02[8];//0x18FEF1A6
	uint8_t _data_03[8];//0x18FEF2A6
	uint8_t _data_04[8];//0x18FEF3A6
	uint8_t _data_05[8];//0x18FEF4A6
	uint8_t _data_06[8];//0x1000A6A9
	uint8_t _data_07[8];//0x1D00A6A9
	uint8_t _data_08[8];//0x0800A6A9
	uint8_t _data_09[8];//0x1C00A6A9
	uint8_t _data_10[8];//0x18FF50E5
	uint8_t _data_11[8];
	uint8_t _data_12[8];
}dykj_can_data_t;


typedef struct
{
	uint8_t _data_01[8];//0x18FEF0A6 
	uint8_t _data_02[8];//0x18FEF1A6
	uint8_t _data_03[8];//0x18FEF2A6
	uint8_t _data_04[8];//0x18FEF3A6
	uint8_t _data_05[8];//0x18FEF4A6
	uint8_t _data_06[8];//0x1000A6A9
	uint8_t _data_07[8];//0x1D00A6A9
	uint8_t _data_08[8];//0x0800A6A9
	uint8_t _data_09[8];//0x1C00A6A9
	uint8_t _data_10[8];//0x18FF50E5
	uint8_t _data_11[8];
	uint8_t _data_12[8];
}jsyt_can_data_t;


typedef struct
{
	uint8_t _data_01[8];//0xCF100D0
	uint8_t _data_02[8];//0xCF101D0
	uint8_t _data_03[8];//0xCF10BD0
	uint8_t _data_04[8];//0x18F43A17	
	uint8_t _data_05[8];//0XCFF0008
	uint8_t _data_06[8];//0XCFF0108
	uint8_t _data_07[8];//0XCFF0208
	uint8_t _data_08[8];//0x18F212F3
	uint8_t _data_09[8];//0x18F213F3
	uint8_t _data_10[8];//0x18F214F3
	uint8_t _data_11[8];//0x181E17F3	
	uint8_t _data_12[8];//0x181F17F3
	uint8_t _data_13[8];//0x182017F3
	uint8_t _data_14[8];//0x18A017F3
	uint8_t _data_15[8];//0x0CF217F3
	uint8_t _data_16[8];//0x18F302E4
	uint8_t _data_17[8];
}vict_can_data_t;

typedef struct
{
	uint8_t _data_01[8];//0x18FF01F1
	uint8_t _data_02[8];//0x18FF02F1
	uint8_t _data_03[8];//0x18FF03F1
	uint8_t _data_04[8];//0x18FF04F1	
	uint8_t _data_05[8];//0x18FF05F1
	uint8_t _data_06[8];//0x18FF06F1
	uint8_t _data_07[8];//0x18FF07F1
	uint8_t _data_08[8];//0xC07F0A7
	uint8_t _data_09[8];//0xC08A7F0
	uint8_t _data_10[8];//0xC09A7F0
	uint8_t _data_11[8];//0xC0AA7F0
	uint8_t _data_12[8];//
}ydzy_can_data_t;


extern hy_can_data_t _hy_can_data;
extern ax_can_data_t _ax_can_data;
extern xx_can_data_t _xx_can_data;
extern cgqc_can_data_t _cgqc_can_data;
extern dykj_can_data_t _dykj_can_data;
extern jsyt_can_data_t _jsyt_can_data;
extern vict_can_data_t _vict_can_data;
extern ydzy_can_data_t _ydzy_can_data;

extern uint8_t init_can(void);

#endif
