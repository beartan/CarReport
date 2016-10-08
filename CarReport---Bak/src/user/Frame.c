#include "Frame.h"
#include "Gstar.h"
#include "Can.h"
#include "Sim800.h"
#include "Usart.h"

static uint8_t _report_hy_data[127];
static uint8_t _report_ax_data[131];
static uint8_t _report_xx_data[108];
static uint8_t _report_cgqc_data[145];
static uint8_t _report_dykj_data[145];
static uint8_t _report_jsyt_data[145];
static uint8_t _report_vict_data[185];
static uint8_t _report_ydzy_data[145];



static void _set_hy_frame(void)
{
	uint8_t shi, fen, miao;
	uint8_t pos = 0;
	//初始化
	memset(_report_hy_data, 0 ,sizeof(_report_hy_data));
	
	//车辆VIN - 24 字节
	pos = 0;
	/*01*/_report_hy_data[pos++] = 0x00;
	/*02*/_report_hy_data[pos++] = g_car_config.car_vin[0x00];
	/*03*/_report_hy_data[pos++] = g_car_config.car_vin[0x01];
	/*04*/_report_hy_data[pos++] = g_car_config.car_vin[0x02];
	/*05*/_report_hy_data[pos++] = g_car_config.car_vin[0x03];
	/*06*/_report_hy_data[pos++] = g_car_config.car_vin[0x04];
	/*07*/_report_hy_data[pos++] = g_car_config.car_vin[0x05];
	/*08*/_report_hy_data[pos++] = (_report_hy_data[0x00] + _report_hy_data[0x01] + _report_hy_data[0x02] + _report_hy_data[0x03] + _report_hy_data[0x04] + _report_hy_data[0x05] + _report_hy_data[0x06]) & 0xFF;
	
	/*09*/_report_hy_data[pos++] = 0x01;
	/*10*/_report_hy_data[pos++] = g_car_config.car_vin[0x06];
	/*11*/_report_hy_data[pos++] = g_car_config.car_vin[0x07];
	/*12*/_report_hy_data[pos++] = g_car_config.car_vin[0x08];
	/*13*/_report_hy_data[pos++] = g_car_config.car_vin[0x09];
	/*14*/_report_hy_data[pos++] = g_car_config.car_vin[0x0A];
	/*15*/_report_hy_data[pos++] = g_car_config.car_vin[0x0B];
	/*16*/_report_hy_data[pos++] = (_report_hy_data[0x08] + _report_hy_data[0x09] + _report_hy_data[0x0A] + _report_hy_data[0x0B] + _report_hy_data[0x0C] + _report_hy_data[0x0D] + _report_hy_data[0x0E]) & 0xFF;

	/*17*/_report_hy_data[pos++] = 0x02;
	/*18*/_report_hy_data[pos++] = g_car_config.car_vin[0x0C];
	/*19*/_report_hy_data[pos++] = g_car_config.car_vin[0x0D];
	/*20*/_report_hy_data[pos++] = g_car_config.car_vin[0x0E];
	/*21*/_report_hy_data[pos++] = g_car_config.car_vin[0x0F];
	/*22*/_report_hy_data[pos++] = g_car_config.car_vin[0x10];
	/*23*/_report_hy_data[pos++] = 0x00;
	/*24*/_report_hy_data[pos++] = (_report_hy_data[0x10] + _report_hy_data[0x11] + _report_hy_data[0x12] + _report_hy_data[0x13] + _report_hy_data[0x14] + _report_hy_data[0x15] + _report_hy_data[0x16]) & 0xFF;

	//记录采样时间
	/*25*/_report_hy_data[pos++] = g_ddmmyy % 100;//年
	/*26*/_report_hy_data[pos++] = (g_ddmmyy / 100) % 100;//月
	/*27*/_report_hy_data[pos++] = (g_ddmmyy / 10000) % 100;;//日
	/*28*/_report_hy_data[pos++] = (g_hhmmss / 10000) + 8;//时
	/*29*/_report_hy_data[pos++] = (g_hhmmss / 100) % 100;//分
	/*30*/_report_hy_data[pos++] = g_hhmmss % 100;//秒

	
	//记录采样时间
	/*25*/_report_hy_data[pos++] = g_ddmmyy % 100;//年
	/*26*/_report_hy_data[pos++] = (g_ddmmyy / 100) % 100;//月
	/*27*/_report_hy_data[pos++] = (g_ddmmyy / 10000) % 100;;//日
	shi = (g_hhmmss / 10000) + 8;
	shi = shi % 24;
	/*28*/_report_hy_data[pos++] = shi;//时
	fen = (g_hhmmss / 100) % 100;
	fen = fen % 60;
	/*29*/_report_hy_data[pos++] = fen;//分
	miao = g_hhmmss % 100;
	miao = miao % 60;
	/*30*/_report_hy_data[pos++] = miao;//秒

	debug_printf("%d %d %d %d %d %d\r\n", _report_hy_data[25],_report_hy_data[26],_report_hy_data[27],_report_hy_data[28],_report_hy_data[29],_report_hy_data[30]);

	//GPS定位状态
	/*31*/_report_hy_data[pos++] = g_status;//0000 0001

	//GPS经度信息
	/*32*/_report_hy_data[pos++] = (g_latitude & 0xFF000000) >> 24;
	/*33*/_report_hy_data[pos++] = (g_latitude & 0x00FF0000) >> 16;
	/*34*/_report_hy_data[pos++] = (g_latitude & 0x0000FF00) >> 8;
	/*35*/_report_hy_data[pos++] = (g_latitude & 0x000000FF);

	//GPS纬度信息
	/*36*/_report_hy_data[pos++] = (g_longitude & 0xFF000000) >> 24;
	/*37*/_report_hy_data[pos++] = (g_longitude & 0x00FF0000) >> 16;
	/*38*/_report_hy_data[pos++] = (g_longitude & 0x0000FF00) >> 8;
	/*39*/_report_hy_data[pos++] = (g_longitude & 0x000000FF);

	//GPS速度
	/*41*/_report_hy_data[pos++] = (g_speed & 0xFF00) >> 8;
	/*40*/_report_hy_data[pos++] = (g_speed & 0x00FF);
	

	//GPS方向
	/*43*/_report_hy_data[pos++] = (g_course & 0xFF00) >> 8;
	/*42*/_report_hy_data[pos++] = (g_course & 0x00FF);

	//GPS纬保留信息
	/*44*/_report_hy_data[pos++] = 0;
	/*45*/_report_hy_data[pos++] = 0;
	/*46*/_report_hy_data[pos++] = 0;
	/*47*/_report_hy_data[pos++] = 0;

	//里程数/续航里程
	/*48*/_report_hy_data[pos++] = _hy_can_data._mileage[0];
	/*49*/_report_hy_data[pos++] = _hy_can_data._mileage[1];
	/*50*/_report_hy_data[pos++] = _hy_can_data._mileage[2];
	/*51*/_report_hy_data[pos++] = _hy_can_data._mileage[3];
	/*52*/_report_hy_data[pos++] = _hy_can_data._mileage[4];
	/*53*/_report_hy_data[pos++] = _hy_can_data._mileage[5];
	/*54*/_report_hy_data[pos++] = _hy_can_data._mileage[6];
	/*55*/_report_hy_data[pos++] = _hy_can_data._mileage[7];

	//电池电压/电池电流/车速/电机转速
	/*56*/_report_hy_data[pos++] = _hy_can_data._motor_speed[0];
	/*57*/_report_hy_data[pos++] = _hy_can_data._motor_speed[1];
	/*58*/_report_hy_data[pos++] = _hy_can_data._motor_speed[2];
	/*59*/_report_hy_data[pos++] = _hy_can_data._motor_speed[3];
	/*60*/_report_hy_data[pos++] = _hy_can_data._motor_speed[4];
	/*61*/_report_hy_data[pos++] = _hy_can_data._motor_speed[5];
	/*62*/_report_hy_data[pos++] = _hy_can_data._motor_speed[6];
	/*63*/_report_hy_data[pos++] = _hy_can_data._motor_speed[7];

	//单体电池最高/最低电压
	/*64*/_report_hy_data[pos++] = _hy_can_data._battery[0];
	/*65*/_report_hy_data[pos++] = _hy_can_data._battery[1];
	/*66*/_report_hy_data[pos++] = _hy_can_data._battery[2];
	/*67*/_report_hy_data[pos++] = _hy_can_data._battery[3];
	/*68*/_report_hy_data[pos++] = _hy_can_data._battery[4];
	/*69*/_report_hy_data[pos++] = _hy_can_data._battery[5];
	/*70*/_report_hy_data[pos++] = 0xFF;//固定值
	/*71*/_report_hy_data[pos++] = 0xFF;//固定值

	//SOC/车辆状态/充电起止时间/车辆状态/启动/工作状态信号
	/*72*/_report_hy_data[pos++] = _hy_can_data._car_status[0];
	/*73*/_report_hy_data[pos++] = _hy_can_data._car_status[1];
	/*74*/_report_hy_data[pos++] = _hy_can_data._car_status[2];
	/*75*/_report_hy_data[pos++] = _hy_can_data._car_status[3];
	/*76*/_report_hy_data[pos++] = _hy_can_data._car_status[4];
	/*77*/_report_hy_data[pos++] = _hy_can_data._car_status[5];
	/*78*/_report_hy_data[pos++] = _hy_can_data._car_status[6];
	/*79*/_report_hy_data[pos++] = _hy_can_data._car_status[7];

	//电机转矩
	/*80*/_report_hy_data[pos++] = _hy_can_data._motor_torque[0];
	/*81*/_report_hy_data[pos++] = _hy_can_data._motor_torque[1];
	/*82*/_report_hy_data[pos++] = _hy_can_data._motor_torque[2];
	/*83*/_report_hy_data[pos++] = _hy_can_data._motor_torque[3];
	/*84*/_report_hy_data[pos++] = _hy_can_data._motor_torque[4];
	/*85*/_report_hy_data[pos++] = _hy_can_data._motor_torque[5];
	/*86*/_report_hy_data[pos++] = _hy_can_data._motor_torque[6];
	/*87*/_report_hy_data[pos++] = _hy_can_data._motor_torque[7];

	//驱动电机温度/逆变器温度
	/*88*/_report_hy_data[pos++] = 0xFF;//固定值
	/*89*/_report_hy_data[pos++] = _hy_can_data._motor_temp[1];
	/*90*/_report_hy_data[pos++] = _hy_can_data._motor_temp[2];
	/*91*/_report_hy_data[pos++] = 0xFF;//固定值
	/*92*/_report_hy_data[pos++] = 0xFF;//固定值
	/*93*/_report_hy_data[pos++] = 0xFF;//固定值
	/*94*/_report_hy_data[pos++] = 0xFF;//固定值
	/*95*/_report_hy_data[pos++] = 0xFF;//固定值

	//电机电流/电机电压
	/*96*/_report_hy_data[pos++] = _hy_can_data._motor_current[0];
	/*97*/_report_hy_data[pos++] = _hy_can_data._motor_current[1];
	/*98*/_report_hy_data[pos++] = _hy_can_data._motor_current[2];
	/*99*/_report_hy_data[pos++] = _hy_can_data._motor_current[3];
	/*100*/_report_hy_data[pos++] = _hy_can_data._motor_current[4];
	/*101*/_report_hy_data[pos++] = _hy_can_data._motor_current[5];
	/*102*/_report_hy_data[pos++] = _hy_can_data._motor_current[6];
	/*103*/_report_hy_data[pos++] = 0xFF;//固定值

	//充电电压/充电电流
	/*104*/_report_hy_data[pos++] = _hy_can_data._charge_voltage[0];
	/*105*/_report_hy_data[pos++] = _hy_can_data._charge_voltage[1];
	/*106*/_report_hy_data[pos++] = _hy_can_data._charge_voltage[2];
	/*107*/_report_hy_data[pos++] = _hy_can_data._charge_voltage[3];
	/*108*/_report_hy_data[pos++] = _hy_can_data._charge_voltage[4];
	/*109*/_report_hy_data[pos++] = _hy_can_data._charge_voltage[5];
	/*110*/_report_hy_data[pos++] = _hy_can_data._charge_voltage[6];
	/*111*/_report_hy_data[pos++] = _hy_can_data._charge_voltage[7];

	//绝缘电阻
	/*112*/_report_hy_data[pos++] = _hy_can_data._resistance[0];
	/*113*/_report_hy_data[pos++] = _hy_can_data._resistance[1];
	/*114*/_report_hy_data[pos++] = _hy_can_data._resistance[2];
	/*115*/_report_hy_data[pos++] = _hy_can_data._resistance[3];
	/*116*/_report_hy_data[pos++] = _hy_can_data._resistance[4];
	/*117*/_report_hy_data[pos++] = _hy_can_data._resistance[5];
	/*118*/_report_hy_data[pos++] = _hy_can_data._resistance[6];
	/*119*/_report_hy_data[pos++] = _hy_can_data._resistance[7];

	//故障报警
	/*120*/_report_hy_data[pos++] = 0xFF;//固定值
	/*121*/_report_hy_data[pos++] = 0xFF;//固定值
	/*122*/_report_hy_data[pos++] = _hy_can_data._error_alarm[2];
	/*123*/_report_hy_data[pos++] = _hy_can_data._error_alarm[3];
	/*124*/_report_hy_data[pos++] = _hy_can_data._error_alarm[4];
	/*125*/_report_hy_data[pos++] = _hy_can_data._error_alarm[5];
	/*126*/_report_hy_data[pos++] = 0xFF;//固定值
	/*127*/_report_hy_data[pos++] = 0xFF;//固定值
}


static void _set_ax_frame(void)
{
	uint8_t shi, fen, miao;
	uint8_t pos = 0;
	//初始化
	memset(_report_ax_data, 0 ,sizeof(_report_ax_data));

	//车辆VIN - 24 字节
	pos = 0;
	/*01*/_report_ax_data[pos++] = 0x00;
	/*02*/_report_ax_data[pos++] = g_car_config.car_vin[0x00];
	/*03*/_report_ax_data[pos++] = g_car_config.car_vin[0x01];
	/*04*/_report_ax_data[pos++] = g_car_config.car_vin[0x02];
	/*05*/_report_ax_data[pos++] = g_car_config.car_vin[0x03];
	/*06*/_report_ax_data[pos++] = g_car_config.car_vin[0x04];
	/*07*/_report_ax_data[pos++] = g_car_config.car_vin[0x05];
	/*08*/_report_ax_data[pos++] = (_report_ax_data[0x00] + _report_ax_data[0x01] + _report_ax_data[0x02] + _report_ax_data[0x03] + _report_ax_data[0x04] + _report_ax_data[0x05] + _report_ax_data[0x06]) & 0xFF;
	
	/*09*/_report_ax_data[pos++] = 0x01;
	/*10*/_report_ax_data[pos++] = g_car_config.car_vin[0x06];
	/*11*/_report_ax_data[pos++] = g_car_config.car_vin[0x07];
	/*12*/_report_ax_data[pos++] = g_car_config.car_vin[0x08];
	/*13*/_report_ax_data[pos++] = g_car_config.car_vin[0x09];
	/*14*/_report_ax_data[pos++] = g_car_config.car_vin[0x0A];
	/*15*/_report_ax_data[pos++] = g_car_config.car_vin[0x0B];
	/*16*/_report_ax_data[pos++] = (_report_ax_data[0x08] + _report_ax_data[0x09] + _report_ax_data[0x0A] + _report_ax_data[0x0B] + _report_ax_data[0x0C] + _report_ax_data[0x0D] + _report_ax_data[0x0E]) & 0xFF;

	/*17*/_report_ax_data[pos++] = 0x02;
	/*18*/_report_ax_data[pos++] = g_car_config.car_vin[0x0C];
	/*19*/_report_ax_data[pos++] = g_car_config.car_vin[0x0D];
	/*20*/_report_ax_data[pos++] = g_car_config.car_vin[0x0E];
	/*21*/_report_ax_data[pos++] = g_car_config.car_vin[0x0F];
	/*22*/_report_ax_data[pos++] = g_car_config.car_vin[0x10];
	/*23*/_report_ax_data[pos++] = 0x00;
	/*24*/_report_ax_data[pos++] = (_report_ax_data[0x10] + _report_ax_data[0x11] + _report_ax_data[0x12] + _report_ax_data[0x13] + _report_ax_data[0x14] + _report_ax_data[0x15] + _report_ax_data[0x16]) & 0xFF;

	//记录采样时间
	/*25*/_report_ax_data[pos++] = g_ddmmyy % 100;//年
	/*26*/_report_ax_data[pos++] = (g_ddmmyy / 100) % 100;//月
	/*27*/_report_ax_data[pos++] = (g_ddmmyy / 10000) % 100;;//日
	shi = (g_hhmmss / 10000) + 8;
	shi = shi % 24;
	/*28*/_report_ax_data[pos++] = shi;//时
	fen = (g_hhmmss / 100) % 100;
	fen = fen % 60;
	/*29*/_report_ax_data[pos++] = fen;//分
	miao = g_hhmmss % 100;
	miao = miao % 60;
	/*30*/_report_ax_data[pos++] = miao;//秒

	debug_printf("%d %d %d %d %d %d\r\n", _report_ax_data[25],_report_ax_data[26],_report_ax_data[27],_report_ax_data[28],_report_ax_data[29],_report_ax_data[30]);

	//GPS定位状态
	/*31*/_report_ax_data[pos++] = g_status;//0000 0001

	//GPS经度信息
	/*32*/_report_ax_data[pos++] = (g_latitude & 0xFF000000) >> 24;
	/*33*/_report_ax_data[pos++] = (g_latitude & 0x00FF0000) >> 16;
	/*34*/_report_ax_data[pos++] = (g_latitude & 0x0000FF00) >> 8;
	/*35*/_report_ax_data[pos++] = (g_latitude & 0x000000FF);

	//GPS纬度信息
	/*36*/_report_ax_data[pos++] = (g_longitude & 0xFF000000) >> 24;
	/*37*/_report_ax_data[pos++] = (g_longitude & 0x00FF0000) >> 16;
	/*38*/_report_ax_data[pos++] = (g_longitude & 0x0000FF00) >> 8;
	/*39*/_report_ax_data[pos++] = (g_longitude & 0x000000FF);

	//GPS速度
	/*41*/_report_ax_data[pos++] = (g_speed & 0xFF00) >> 8;
	/*40*/_report_ax_data[pos++] = (g_speed & 0x00FF);

	//GPS方向
	/*43*/_report_ax_data[pos++] = (g_course & 0xFF00) >> 8;
	/*42*/_report_ax_data[pos++] = (g_course & 0x00FF);

	//GPS纬保留信息
	/*44*/_report_ax_data[pos++] = 0;
	/*45*/_report_ax_data[pos++] = 0;
	/*46*/_report_ax_data[pos++] = 0;
	/*47*/_report_ax_data[pos++] = 0;

	/*48*/_report_ax_data[pos++] = _ax_can_data._data_01[0];
	/*49*/_report_ax_data[pos++] = _ax_can_data._data_01[1];
	/*50*/_report_ax_data[pos++] = _ax_can_data._data_01[2];
	/*51*/_report_ax_data[pos++] = _ax_can_data._data_01[3];
	/*52*/_report_ax_data[pos++] = _ax_can_data._data_01[4];
	/*53*/_report_ax_data[pos++] = _ax_can_data._data_01[5];
	/*54*/_report_ax_data[pos++] = _ax_can_data._data_01[6];

	/*55*/_report_ax_data[pos++] = _ax_can_data._data_02[0];
	/*56*/_report_ax_data[pos++] = _ax_can_data._data_02[1];
	/*57*/_report_ax_data[pos++] = _ax_can_data._data_02[2];
	/*58*/_report_ax_data[pos++] = _ax_can_data._data_02[3];

	/*59*/_report_ax_data[pos++] = _ax_can_data._data_03[0];
	/*60*/_report_ax_data[pos++] = _ax_can_data._data_03[1];
	/*61*/_report_ax_data[pos++] = _ax_can_data._data_03[2];
	/*62*/_report_ax_data[pos++] = _ax_can_data._data_03[3];
	/*63*/_report_ax_data[pos++] = _ax_can_data._data_03[4];
	/*64*/_report_ax_data[pos++] = _ax_can_data._data_03[5];
	/*65*/_report_ax_data[pos++] = _ax_can_data._data_03[6];
	/*66*/_report_ax_data[pos++] = _ax_can_data._data_03[7];

	/*67*/_report_ax_data[pos++] = _ax_can_data._data_04[0];
	/*68*/_report_ax_data[pos++] = _ax_can_data._data_04[1];
	/*69*/_report_ax_data[pos++] = _ax_can_data._data_04[2];
	/*70*/_report_ax_data[pos++] = _ax_can_data._data_04[3];
	/*71*/_report_ax_data[pos++] = _ax_can_data._data_04[4];
	/*72*/_report_ax_data[pos++] = _ax_can_data._data_04[5];
	/*73*/_report_ax_data[pos++] = _ax_can_data._data_04[6];
	/*74*/_report_ax_data[pos++] = _ax_can_data._data_04[7];

	/*75*/_report_ax_data[pos++] = _ax_can_data._data_05[0];

	/*76*/_report_ax_data[pos++] = _ax_can_data._data_06[0];
	/*77*/_report_ax_data[pos++] = _ax_can_data._data_06[1];
	/*78*/_report_ax_data[pos++] = _ax_can_data._data_06[2];
	/*79*/_report_ax_data[pos++] = _ax_can_data._data_06[3];
	/*80*/_report_ax_data[pos++] = _ax_can_data._data_06[4];
	/*81*/_report_ax_data[pos++] = _ax_can_data._data_06[5];
	/*82*/_report_ax_data[pos++] = _ax_can_data._data_06[6];
	/*83*/_report_ax_data[pos++] = _ax_can_data._data_06[7];

	/*84*/_report_ax_data[pos++] = _ax_can_data._data_09[0];
	/*85*/_report_ax_data[pos++] = _ax_can_data._data_09[1];
	/*86*/_report_ax_data[pos++] = _ax_can_data._data_09[2];
	/*87*/_report_ax_data[pos++] = _ax_can_data._data_09[3];

	/*88*/_report_ax_data[pos++] = _ax_can_data._data_10[0];
	/*89*/_report_ax_data[pos++] = _ax_can_data._data_10[1];
	/*90*/_report_ax_data[pos++] = _ax_can_data._data_10[2];
	/*91*/_report_ax_data[pos++] = _ax_can_data._data_10[3];
	/*92*/_report_ax_data[pos++] = _ax_can_data._data_10[4];
	/*93*/_report_ax_data[pos++] = _ax_can_data._data_10[5];
	/*94*/_report_ax_data[pos++] = _ax_can_data._data_10[6];
	/*95*/_report_ax_data[pos++] = _ax_can_data._data_10[7];

	/*96*/_report_ax_data[pos++] = _ax_can_data._data_11[0];
	/*97*/_report_ax_data[pos++] = _ax_can_data._data_11[1];
	/*98*/_report_ax_data[pos++] = _ax_can_data._data_11[2];
	/*99*/_report_ax_data[pos++] = _ax_can_data._data_11[3];
	/*100*/_report_ax_data[pos++] = _ax_can_data._data_11[4];
	/*101*/_report_ax_data[pos++] = _ax_can_data._data_11[5];
	/*102*/_report_ax_data[pos++] = _ax_can_data._data_11[6];
	/*103*/_report_ax_data[pos++] = _ax_can_data._data_11[7];

	/*104*/_report_ax_data[pos++] = _ax_can_data._data_12[0];
	/*105*/_report_ax_data[pos++] = _ax_can_data._data_12[1];
	/*106*/_report_ax_data[pos++] = _ax_can_data._data_12[2];
	/*107*/_report_ax_data[pos++] = _ax_can_data._data_12[3];
	/*108*/_report_ax_data[pos++] = _ax_can_data._data_12[4];

	/*109*/_report_ax_data[pos++] = _ax_can_data._data_13[0];
	/*110*/_report_ax_data[pos++] = _ax_can_data._data_13[1];
	/*111*/_report_ax_data[pos++] = _ax_can_data._data_13[2];
	/*112*/_report_ax_data[pos++] = _ax_can_data._data_13[3];

	/*113*/_report_ax_data[pos++] = _ax_can_data._data_14[0];
	/*114*/_report_ax_data[pos++] = _ax_can_data._data_14[1];
	/*115*/_report_ax_data[pos++] = _ax_can_data._data_14[2];
	/*116*/_report_ax_data[pos++] = _ax_can_data._data_14[3];
	/*117*/_report_ax_data[pos++] = _ax_can_data._data_14[4];
	/*118*/_report_ax_data[pos++] = _ax_can_data._data_14[5];
	/*119*/_report_ax_data[pos++] = _ax_can_data._data_14[6];
	/*120*/_report_ax_data[pos++] = _ax_can_data._data_14[7];

	/*121*/_report_ax_data[pos++] = _ax_can_data._data_15[0];
	/*122*/_report_ax_data[pos++] = _ax_can_data._data_15[1];
	/*123*/_report_ax_data[pos++] = _ax_can_data._data_15[2];
	/*124*/_report_ax_data[pos++] = _ax_can_data._data_15[3];
	/*125*/_report_ax_data[pos++] = _ax_can_data._data_15[4];
	/*126*/_report_ax_data[pos++] = _ax_can_data._data_15[5];

	/*127*/_report_ax_data[pos++] = _ax_can_data._data_16[0];
	/*128*/_report_ax_data[pos++] = _ax_can_data._data_16[1];
	/*129*/_report_ax_data[pos++] = _ax_can_data._data_16[2];
	/*130*/_report_ax_data[pos++] = _ax_can_data._data_16[3];
	/*131*/_report_ax_data[pos++] = _ax_can_data._data_16[4];
}



static void _set_xx_frame(void)
{
	uint8_t shi, fen, miao;
	uint8_t pos = 0;
	//初始化
	memset(_report_xx_data, 0 ,sizeof(_report_xx_data));

	//车辆VIN - 24 字节
	pos = 0;
	/*01*/_report_xx_data[pos++] = 0x00;
	/*02*/_report_xx_data[pos++] = g_car_config.car_vin[0x00];
	/*03*/_report_xx_data[pos++] = g_car_config.car_vin[0x01];
	/*04*/_report_xx_data[pos++] = g_car_config.car_vin[0x02];
	/*05*/_report_xx_data[pos++] = g_car_config.car_vin[0x03];
	/*06*/_report_xx_data[pos++] = g_car_config.car_vin[0x04];
	/*07*/_report_xx_data[pos++] = g_car_config.car_vin[0x05];
	/*08*/_report_xx_data[pos++] = (_report_xx_data[0x00] + _report_xx_data[0x01] + _report_xx_data[0x02] + _report_xx_data[0x03] + _report_xx_data[0x04] + _report_xx_data[0x05] + _report_xx_data[0x06]) & 0xFF;
	
	/*09*/_report_xx_data[pos++] = 0x01;
	/*10*/_report_xx_data[pos++] = g_car_config.car_vin[0x06];
	/*11*/_report_xx_data[pos++] = g_car_config.car_vin[0x07];
	/*12*/_report_xx_data[pos++] = g_car_config.car_vin[0x08];
	/*13*/_report_xx_data[pos++] = g_car_config.car_vin[0x09];
	/*14*/_report_xx_data[pos++] = g_car_config.car_vin[0x0A];
	/*15*/_report_xx_data[pos++] = g_car_config.car_vin[0x0B];
	/*16*/_report_xx_data[pos++] = (_report_xx_data[0x08] + _report_xx_data[0x09] + _report_xx_data[0x0A] + _report_xx_data[0x0B] + _report_xx_data[0x0C] + _report_xx_data[0x0D] + _report_xx_data[0x0E]) & 0xFF;

	/*17*/_report_xx_data[pos++] = 0x02;
	/*18*/_report_xx_data[pos++] = g_car_config.car_vin[0x0C];
	/*19*/_report_xx_data[pos++] = g_car_config.car_vin[0x0D];
	/*20*/_report_xx_data[pos++] = g_car_config.car_vin[0x0E];
	/*21*/_report_xx_data[pos++] = g_car_config.car_vin[0x0F];
	/*22*/_report_xx_data[pos++] = g_car_config.car_vin[0x10];
	/*23*/_report_xx_data[pos++] = 0x00;
	/*24*/_report_xx_data[pos++] = (_report_xx_data[0x10] + _report_xx_data[0x11] + _report_xx_data[0x12] + _report_xx_data[0x13] + _report_xx_data[0x14] + _report_xx_data[0x15] + _report_xx_data[0x16]) & 0xFF;

	//记录采样时间
	/*25*/_report_xx_data[pos++] = g_ddmmyy % 100;//年
	/*26*/_report_xx_data[pos++] = (g_ddmmyy / 100) % 100;//月
	/*27*/_report_xx_data[pos++] = (g_ddmmyy / 10000) % 100;;//日
	shi = (g_hhmmss / 10000) + 8;
	shi = shi % 24;
	/*28*/_report_xx_data[pos++] = shi;//时
	fen = (g_hhmmss / 100) % 100;
	fen = fen % 60;
	/*29*/_report_xx_data[pos++] = fen;//分
	miao = g_hhmmss % 100;
	miao = miao % 60;
	/*30*/_report_xx_data[pos++] = miao;//秒

	debug_printf("%d %d %d %d %d %d\r\n", _report_xx_data[25],_report_xx_data[26],_report_xx_data[27],_report_xx_data[28],_report_xx_data[29],_report_xx_data[30]);

	//GPS定位状态
	/*31*/_report_xx_data[pos++] = g_status;//0000 0001

	//GPS经度信息
	/*32*/_report_xx_data[pos++] = (g_latitude & 0xFF000000) >> 24;
	/*33*/_report_xx_data[pos++] = (g_latitude & 0x00FF0000) >> 16;
	/*34*/_report_xx_data[pos++] = (g_latitude & 0x0000FF00) >> 8;
	/*35*/_report_xx_data[pos++] = (g_latitude & 0x000000FF);

	//GPS纬度信息
	/*36*/_report_xx_data[pos++] = (g_longitude & 0xFF000000) >> 24;
	/*37*/_report_xx_data[pos++] = (g_longitude & 0x00FF0000) >> 16;
	/*38*/_report_xx_data[pos++] = (g_longitude & 0x0000FF00) >> 8;
	/*39*/_report_xx_data[pos++] = (g_longitude & 0x000000FF);

	//GPS速度
	/*41*/_report_xx_data[pos++] = (g_speed & 0xFF00) >> 8;
	/*40*/_report_xx_data[pos++] = (g_speed & 0x00FF);

	//GPS方向
	/*43*/_report_xx_data[pos++] = (g_course & 0xFF00) >> 8;
	/*42*/_report_xx_data[pos++] = (g_course & 0x00FF);

	//GPS纬保留信息
	/*44*/_report_xx_data[pos++] = 0;
	/*45*/_report_xx_data[pos++] = 0;
	/*46*/_report_xx_data[pos++] = 0;
	/*47*/_report_xx_data[pos++] = 0;

	/*48*/_report_xx_data[pos++] = _xx_can_data._data_01[0];
	/*49*/_report_xx_data[pos++] = _xx_can_data._data_01[1];
	/*50*/_report_xx_data[pos++] = _xx_can_data._data_01[2];
	/*51*/_report_xx_data[pos++] = _xx_can_data._data_01[3];
	/*52*/_report_xx_data[pos++] = _xx_can_data._data_01[4];
	/*53*/_report_xx_data[pos++] = _xx_can_data._data_01[5];
	/*54*/_report_xx_data[pos++] = _xx_can_data._data_01[6];

	/*55*/_report_xx_data[pos++] = _xx_can_data._data_02[0];
	/*56*/_report_xx_data[pos++] = _xx_can_data._data_02[1];
	/*57*/_report_xx_data[pos++] = _xx_can_data._data_02[2];
	/*58*/_report_xx_data[pos++] = _xx_can_data._data_02[3];
	/*59*/_report_xx_data[pos++] = _xx_can_data._data_02[4];
	/*60*/_report_xx_data[pos++] = _xx_can_data._data_02[5];
	/*61*/_report_xx_data[pos++] = _xx_can_data._data_02[6];
	
	/*62*/_report_xx_data[pos++] = _xx_can_data._data_03[0];
	/*63*/_report_xx_data[pos++] = _xx_can_data._data_03[1];
	/*64*/_report_xx_data[pos++] = _xx_can_data._data_03[2];
	/*65*/_report_xx_data[pos++] = _xx_can_data._data_03[3];
	/*66*/_report_xx_data[pos++] = _xx_can_data._data_03[4];
	/*67*/_report_xx_data[pos++] = _xx_can_data._data_03[5];
	
	/*68*/_report_xx_data[pos++] = _xx_can_data._data_04[0];
	/*69*/_report_xx_data[pos++] = _xx_can_data._data_04[1];
	/*70*/_report_xx_data[pos++] = _xx_can_data._data_04[2];
	/*71*/_report_xx_data[pos++] = _xx_can_data._data_04[3];
	/*72*/_report_xx_data[pos++] = _xx_can_data._data_04[4];
	/*73*/_report_xx_data[pos++] = _xx_can_data._data_04[5];
	/*74*/_report_xx_data[pos++] = _xx_can_data._data_04[6];

	/*75*/_report_xx_data[pos++] = _xx_can_data._data_05[0];
	/*76*/_report_xx_data[pos++] = _xx_can_data._data_05[1];
	/*77*/_report_xx_data[pos++] = _xx_can_data._data_05[2];
	/*78*/_report_xx_data[pos++] = _xx_can_data._data_05[3];
	/*79*/_report_xx_data[pos++] = _xx_can_data._data_05[4];
	/*80*/_report_xx_data[pos++] = _xx_can_data._data_05[5];
	/*81*/_report_xx_data[pos++] = _xx_can_data._data_05[6];
	
	/*82*/_report_xx_data[pos++] = _xx_can_data._data_06[0];
	/*83*/_report_xx_data[pos++] = _xx_can_data._data_06[1];

	/*84*/_report_xx_data[pos++] = _xx_can_data._data_07[0];
	/*85*/_report_xx_data[pos++] = _xx_can_data._data_07[1];
	
	/*86*/_report_xx_data[pos++] = _xx_can_data._data_08[0];
	/*87*/_report_xx_data[pos++] = _xx_can_data._data_08[1];
	/*88*/_report_xx_data[pos++] = _xx_can_data._data_08[2];
	/*89*/_report_xx_data[pos++] = _xx_can_data._data_08[3];
	/*90*/_report_xx_data[pos++] = _xx_can_data._data_08[4];
	/*91*/_report_xx_data[pos++] = _xx_can_data._data_08[5];
	/*92*/_report_xx_data[pos++] = _xx_can_data._data_08[6];
	
	/*93*/_report_xx_data[pos++] = _xx_can_data._data_09[0];
	/*94*/_report_xx_data[pos++] = _xx_can_data._data_09[1];
	/*95*/_report_xx_data[pos++] = _xx_can_data._data_09[2];
	/*96*/_report_xx_data[pos++] = _xx_can_data._data_09[3];
	/*97*/_report_xx_data[pos++] = _xx_can_data._data_09[4];
	/*98*/_report_xx_data[pos++] = _xx_can_data._data_09[5];
	
	/*99*/_report_xx_data[pos++] = _xx_can_data._data_10[0];
	/*100*/_report_xx_data[pos++] = _xx_can_data._data_10[1];
	/*101*/_report_xx_data[pos++] = _xx_can_data._data_10[2];
	/*102*/_report_xx_data[pos++] = _xx_can_data._data_10[3];
	
	/*103*/_report_xx_data[pos++] = _xx_can_data._data_11[0];
	/*104*/_report_xx_data[pos++] = _xx_can_data._data_11[1];
	/*105*/_report_xx_data[pos++] = _xx_can_data._data_11[2];
	/*106*/_report_xx_data[pos++] = _xx_can_data._data_11[3];
	
	/*107*/_report_xx_data[pos++] = _xx_can_data._data_12[0];
	/*108*/_report_xx_data[pos++] = _xx_can_data._data_12[1];
}


static void _set_cgqc_frame(void)
{
	uint8_t shi, fen, miao;
	uint8_t pos = 0;
	//初始化
	memset(_report_cgqc_data, 0 ,sizeof(_report_cgqc_data));

	//车辆VIN - 24 字节
	pos = 0;
	/*01*/_report_cgqc_data[pos++] = 0x00;
	/*02*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x00];
	/*03*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x01];
	/*04*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x02];
	/*05*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x03];
	/*06*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x04];
	/*07*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x05];
	/*08*/_report_cgqc_data[pos++] = (_report_cgqc_data[0x00] + _report_cgqc_data[0x01] + _report_cgqc_data[0x02] + _report_cgqc_data[0x03] + _report_cgqc_data[0x04] + _report_cgqc_data[0x05] + _report_cgqc_data[0x06]) & 0xFF;
	
	/*09*/_report_cgqc_data[pos++] = 0x01;
	/*10*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x06];
	/*11*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x07];
	/*12*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x08];
	/*13*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x09];
	/*14*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x0A];
	/*15*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x0B];
	/*16*/_report_cgqc_data[pos++] = (_report_cgqc_data[0x08] + _report_cgqc_data[0x09] + _report_cgqc_data[0x0A] + _report_cgqc_data[0x0B] + _report_cgqc_data[0x0C] + _report_cgqc_data[0x0D] + _report_cgqc_data[0x0E]) & 0xFF;

	/*17*/_report_cgqc_data[pos++] = 0x02;
	/*18*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x0C];
	/*19*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x0D];
	/*20*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x0E];
	/*21*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x0F];
	/*22*/_report_cgqc_data[pos++] = g_car_config.car_vin[0x10];
	/*23*/_report_cgqc_data[pos++] = 0x00;
	/*24*/_report_cgqc_data[pos++] = (_report_cgqc_data[0x10] + _report_cgqc_data[0x11] + _report_cgqc_data[0x12] + _report_cgqc_data[0x13] + _report_cgqc_data[0x14] + _report_cgqc_data[0x15] + _report_cgqc_data[0x16]) & 0xFF;

	//记录采样时间
	/*25*/_report_cgqc_data[pos++] = g_ddmmyy % 100;//年
	/*26*/_report_cgqc_data[pos++] = (g_ddmmyy / 100) % 100;//月
	/*27*/_report_cgqc_data[pos++] = (g_ddmmyy / 10000) % 100;;//日
	shi = (g_hhmmss / 10000) + 8;
	shi = shi % 24;
	/*28*/_report_cgqc_data[pos++] = shi;//时
	fen = (g_hhmmss / 100) % 100;
	fen = fen % 60;
	/*29*/_report_cgqc_data[pos++] = fen;//分
	miao = g_hhmmss % 100;
	miao = miao % 60;
	/*30*/_report_cgqc_data[pos++] = miao;//秒

	debug_printf("%d %d %d %d %d %d\r\n", _report_cgqc_data[25],_report_cgqc_data[26],_report_cgqc_data[27],_report_cgqc_data[28],_report_cgqc_data[29],_report_cgqc_data[30]);

	//GPS定位状态
	/*31*/_report_cgqc_data[pos++] = g_status;//0000 0001

	//GPS经度信息
	/*32*/_report_cgqc_data[pos++] = (g_latitude & 0xFF000000) >> 24;
	/*33*/_report_cgqc_data[pos++] = (g_latitude & 0x00FF0000) >> 16;
	/*34*/_report_cgqc_data[pos++] = (g_latitude & 0x0000FF00) >> 8;
	/*35*/_report_cgqc_data[pos++] = (g_latitude & 0x000000FF);

	//GPS纬度信息
	/*36*/_report_cgqc_data[pos++] = (g_longitude & 0xFF000000) >> 24;
	/*37*/_report_cgqc_data[pos++] = (g_longitude & 0x00FF0000) >> 16;
	/*38*/_report_cgqc_data[pos++] = (g_longitude & 0x0000FF00) >> 8;
	/*39*/_report_cgqc_data[pos++] = (g_longitude & 0x000000FF);

	//GPS速度
	/*41*/_report_cgqc_data[pos++] = (g_speed & 0xFF00) >> 8;
	/*40*/_report_cgqc_data[pos++] = (g_speed & 0x00FF);

	//GPS方向
	/*43*/_report_cgqc_data[pos++] = (g_course & 0xFF00) >> 8;
	/*42*/_report_cgqc_data[pos++] = (g_course & 0x00FF);

	//GPS纬保留信息
	/*44*/_report_cgqc_data[pos++] = 0;
	/*45*/_report_cgqc_data[pos++] = 0;
	/*46*/_report_cgqc_data[pos++] = 0;
	/*47*/_report_cgqc_data[pos++] = 0;


	/*48*/_report_cgqc_data[pos++] = 0;
	/*49*/_report_cgqc_data[pos++] = 0;

	/*50*/_report_cgqc_data[pos++] = _cgqc_can_data._data_01[0];
	/*51*/_report_cgqc_data[pos++] = _cgqc_can_data._data_01[1];
	/*52*/_report_cgqc_data[pos++] = _cgqc_can_data._data_01[2];
	/*53*/_report_cgqc_data[pos++] = _cgqc_can_data._data_01[3];
	/*54*/_report_cgqc_data[pos++] = _cgqc_can_data._data_01[4];
	/*55*/_report_cgqc_data[pos++] = _cgqc_can_data._data_01[5];
	/*56*/_report_cgqc_data[pos++] = _cgqc_can_data._data_01[6];
	/*57*/_report_cgqc_data[pos++] = _cgqc_can_data._data_01[7];

	/*58*/_report_cgqc_data[pos++] = _cgqc_can_data._data_02[0];
	/*59*/_report_cgqc_data[pos++] = _cgqc_can_data._data_02[1];
	/*60*/_report_cgqc_data[pos++] = _cgqc_can_data._data_02[2];
	/*61*/_report_cgqc_data[pos++] = _cgqc_can_data._data_02[3];
	/*62*/_report_cgqc_data[pos++] = _cgqc_can_data._data_02[4];
	/*63*/_report_cgqc_data[pos++] = _cgqc_can_data._data_02[5];
	/*64*/_report_cgqc_data[pos++] = _cgqc_can_data._data_02[6];
	/*65*/_report_cgqc_data[pos++] = _cgqc_can_data._data_02[7];

	/*66*/_report_cgqc_data[pos++] = _cgqc_can_data._data_03[0];
	/*67*/_report_cgqc_data[pos++] = _cgqc_can_data._data_03[1];
	/*68*/_report_cgqc_data[pos++] = _cgqc_can_data._data_03[2];
	/*69*/_report_cgqc_data[pos++] = _cgqc_can_data._data_03[3];
	/*70*/_report_cgqc_data[pos++] = _cgqc_can_data._data_03[4];
	/*71*/_report_cgqc_data[pos++] = _cgqc_can_data._data_03[5];
	/*72*/_report_cgqc_data[pos++] = _cgqc_can_data._data_03[6];
	/*73*/_report_cgqc_data[pos++] = _cgqc_can_data._data_03[7];

	/*74*/_report_cgqc_data[pos++] = _cgqc_can_data._data_04[0];
	/*75*/_report_cgqc_data[pos++] = _cgqc_can_data._data_04[1];
	/*76*/_report_cgqc_data[pos++] = _cgqc_can_data._data_04[2];
	/*77*/_report_cgqc_data[pos++] = _cgqc_can_data._data_04[3];
	/*78*/_report_cgqc_data[pos++] = _cgqc_can_data._data_04[4];
	/*79*/_report_cgqc_data[pos++] = _cgqc_can_data._data_04[5];
	/*80*/_report_cgqc_data[pos++] = _cgqc_can_data._data_04[6];
	/*81*/_report_cgqc_data[pos++] = _cgqc_can_data._data_04[7];

	/*82*/_report_cgqc_data[pos++] = _cgqc_can_data._data_05[0];
	/*83*/_report_cgqc_data[pos++] = _cgqc_can_data._data_05[1];
	/*84*/_report_cgqc_data[pos++] = _cgqc_can_data._data_05[2];
	/*85*/_report_cgqc_data[pos++] = _cgqc_can_data._data_05[3];
	/*86*/_report_cgqc_data[pos++] = _cgqc_can_data._data_05[4];
	/*87*/_report_cgqc_data[pos++] = _cgqc_can_data._data_05[5];
	/*88*/_report_cgqc_data[pos++] = _cgqc_can_data._data_05[6];
	/*89*/_report_cgqc_data[pos++] = _cgqc_can_data._data_05[7];

	/*90*/_report_cgqc_data[pos++] = _cgqc_can_data._data_06[0];
	/*91*/_report_cgqc_data[pos++] = _cgqc_can_data._data_06[1];
	/*92*/_report_cgqc_data[pos++] = _cgqc_can_data._data_06[2];
	/*93*/_report_cgqc_data[pos++] = _cgqc_can_data._data_06[3];
	/*94*/_report_cgqc_data[pos++] = _cgqc_can_data._data_06[4];
	/*95*/_report_cgqc_data[pos++] = _cgqc_can_data._data_06[5];
	/*96*/_report_cgqc_data[pos++] = _cgqc_can_data._data_06[6];
	/*97*/_report_cgqc_data[pos++] = _cgqc_can_data._data_06[7];

	/*98*/_report_cgqc_data[pos++] = _cgqc_can_data._data_07[0];
	/*99*/_report_cgqc_data[pos++] = _cgqc_can_data._data_07[1];
	/*100*/_report_cgqc_data[pos++] = _cgqc_can_data._data_07[2];
	/*101*/_report_cgqc_data[pos++] = _cgqc_can_data._data_07[3];
	/*102*/_report_cgqc_data[pos++] = _cgqc_can_data._data_07[4];
	/*103*/_report_cgqc_data[pos++] = _cgqc_can_data._data_07[5];
	/*104*/_report_cgqc_data[pos++] = _cgqc_can_data._data_07[6];
	/*105*/_report_cgqc_data[pos++] = _cgqc_can_data._data_07[7];

	/*106*/_report_cgqc_data[pos++] = _cgqc_can_data._data_08[0];
	/*107*/_report_cgqc_data[pos++] = _cgqc_can_data._data_08[1];
	/*108*/_report_cgqc_data[pos++] = _cgqc_can_data._data_08[2];
	/*109*/_report_cgqc_data[pos++] = _cgqc_can_data._data_08[3];
	/*110*/_report_cgqc_data[pos++] = _cgqc_can_data._data_08[4];
	/*111*/_report_cgqc_data[pos++] = _cgqc_can_data._data_08[5];
	/*112*/_report_cgqc_data[pos++] = _cgqc_can_data._data_08[6];
	/*113*/_report_cgqc_data[pos++] = _cgqc_can_data._data_08[7];

	/*114*/_report_cgqc_data[pos++] = _cgqc_can_data._data_09[0];
	/*115*/_report_cgqc_data[pos++] = _cgqc_can_data._data_09[1];
	/*116*/_report_cgqc_data[pos++] = _cgqc_can_data._data_09[2];
	/*117*/_report_cgqc_data[pos++] = _cgqc_can_data._data_09[3];
	/*118*/_report_cgqc_data[pos++] = _cgqc_can_data._data_09[4];
	/*119*/_report_cgqc_data[pos++] = _cgqc_can_data._data_09[5];
	/*120*/_report_cgqc_data[pos++] = _cgqc_can_data._data_09[6];
	/*121*/_report_cgqc_data[pos++] = _cgqc_can_data._data_09[7];

	/*122*/_report_cgqc_data[pos++] = _cgqc_can_data._data_10[0];
	/*123*/_report_cgqc_data[pos++] = _cgqc_can_data._data_10[1];
	/*124*/_report_cgqc_data[pos++] = _cgqc_can_data._data_10[2];
	/*125*/_report_cgqc_data[pos++] = _cgqc_can_data._data_10[3];
	/*126*/_report_cgqc_data[pos++] = _cgqc_can_data._data_10[4];
	/*127*/_report_cgqc_data[pos++] = _cgqc_can_data._data_10[5];
	/*128*/_report_cgqc_data[pos++] = _cgqc_can_data._data_10[6];
	/*129*/_report_cgqc_data[pos++] = _cgqc_can_data._data_10[7];

	/*130*/_report_cgqc_data[pos++] = 0;
	/*131*/_report_cgqc_data[pos++] = 0;
	/*132*/_report_cgqc_data[pos++] = 0;
	/*133*/_report_cgqc_data[pos++] = 0;
	/*134*/_report_cgqc_data[pos++] = 0;
	/*135*/_report_cgqc_data[pos++] = 0;
	/*136*/_report_cgqc_data[pos++] = 0;
	/*137*/_report_cgqc_data[pos++] = 0;

	/*138*/_report_cgqc_data[pos++] = 0;
	/*139*/_report_cgqc_data[pos++] = 0;
	/*140*/_report_cgqc_data[pos++] = 0;
	/*141*/_report_cgqc_data[pos++] = 0;
	/*142*/_report_cgqc_data[pos++] = 0;
	/*143*/_report_cgqc_data[pos++] = 0;
	/*144*/_report_cgqc_data[pos++] = 0;
	/*145*/_report_cgqc_data[pos++] = 0;
}


static void _set_dykj_frame(void)
{
	uint8_t shi, fen, miao;
	uint8_t pos = 0;
	//初始化
	memset(_report_dykj_data, 0 ,sizeof(_report_dykj_data));

	//车辆VIN - 24 字节
	pos = 0;
	/*01*/_report_dykj_data[pos++] = 0x00;
	/*02*/_report_dykj_data[pos++] = g_car_config.car_vin[0x00];
	/*03*/_report_dykj_data[pos++] = g_car_config.car_vin[0x01];
	/*04*/_report_dykj_data[pos++] = g_car_config.car_vin[0x02];
	/*05*/_report_dykj_data[pos++] = g_car_config.car_vin[0x03];
	/*06*/_report_dykj_data[pos++] = g_car_config.car_vin[0x04];
	/*07*/_report_dykj_data[pos++] = g_car_config.car_vin[0x05];
	/*08*/_report_dykj_data[pos++] = (_report_dykj_data[0x00] + _report_dykj_data[0x01] + _report_dykj_data[0x02] + _report_dykj_data[0x03] + _report_dykj_data[0x04] + _report_dykj_data[0x05] + _report_dykj_data[0x06]) & 0xFF;
	
	/*09*/_report_dykj_data[pos++] = 0x01;
	/*10*/_report_dykj_data[pos++] = g_car_config.car_vin[0x06];
	/*11*/_report_dykj_data[pos++] = g_car_config.car_vin[0x07];
	/*12*/_report_dykj_data[pos++] = g_car_config.car_vin[0x08];
	/*13*/_report_dykj_data[pos++] = g_car_config.car_vin[0x09];
	/*14*/_report_dykj_data[pos++] = g_car_config.car_vin[0x0A];
	/*15*/_report_dykj_data[pos++] = g_car_config.car_vin[0x0B];
	/*16*/_report_dykj_data[pos++] = (_report_dykj_data[0x08] + _report_dykj_data[0x09] + _report_dykj_data[0x0A] + _report_dykj_data[0x0B] + _report_dykj_data[0x0C] + _report_dykj_data[0x0D] + _report_dykj_data[0x0E]) & 0xFF;

	/*17*/_report_dykj_data[pos++] = 0x02;
	/*18*/_report_dykj_data[pos++] = g_car_config.car_vin[0x0C];
	/*19*/_report_dykj_data[pos++] = g_car_config.car_vin[0x0D];
	/*20*/_report_dykj_data[pos++] = g_car_config.car_vin[0x0E];
	/*21*/_report_dykj_data[pos++] = g_car_config.car_vin[0x0F];
	/*22*/_report_dykj_data[pos++] = g_car_config.car_vin[0x10];
	/*23*/_report_dykj_data[pos++] = 0x00;
	/*24*/_report_dykj_data[pos++] = (_report_dykj_data[0x10] + _report_dykj_data[0x11] + _report_dykj_data[0x12] + _report_dykj_data[0x13] + _report_dykj_data[0x14] + _report_dykj_data[0x15] + _report_dykj_data[0x16]) & 0xFF;

	//记录采样时间
	/*25*/_report_dykj_data[pos++] = g_ddmmyy % 100;//年
	/*26*/_report_dykj_data[pos++] = (g_ddmmyy / 100) % 100;//月
	/*27*/_report_dykj_data[pos++] = (g_ddmmyy / 10000) % 100;;//日
	shi = (g_hhmmss / 10000) + 8;
	shi = shi % 24;
	/*28*/_report_dykj_data[pos++] = shi;//时
	fen = (g_hhmmss / 100) % 100;
	fen = fen % 60;
	/*29*/_report_dykj_data[pos++] = fen;//分
	miao = g_hhmmss % 100;
	miao = miao % 60;
	/*30*/_report_dykj_data[pos++] = miao;//秒

	debug_printf("%d %d %d %d %d %d\r\n", _report_dykj_data[25],_report_dykj_data[26],_report_dykj_data[27],_report_dykj_data[28],_report_dykj_data[29],_report_dykj_data[30]);

	//GPS定位状态
	/*31*/_report_dykj_data[pos++] = g_status;//0000 0001

	//GPS经度信息
	/*32*/_report_dykj_data[pos++] = (g_latitude & 0xFF000000) >> 24;
	/*33*/_report_dykj_data[pos++] = (g_latitude & 0x00FF0000) >> 16;
	/*34*/_report_dykj_data[pos++] = (g_latitude & 0x0000FF00) >> 8;
	/*35*/_report_dykj_data[pos++] = (g_latitude & 0x000000FF);

	//GPS纬度信息
	/*36*/_report_dykj_data[pos++] = (g_longitude & 0xFF000000) >> 24;
	/*37*/_report_dykj_data[pos++] = (g_longitude & 0x00FF0000) >> 16;
	/*38*/_report_dykj_data[pos++] = (g_longitude & 0x0000FF00) >> 8;
	/*39*/_report_dykj_data[pos++] = (g_longitude & 0x000000FF);

	//GPS速度
	/*41*/_report_dykj_data[pos++] = (g_speed & 0xFF00) >> 8;
	/*40*/_report_dykj_data[pos++] = (g_speed & 0x00FF);

	//GPS方向
	/*43*/_report_dykj_data[pos++] = (g_course & 0xFF00) >> 8;
	/*42*/_report_dykj_data[pos++] = (g_course & 0x00FF);

	//GPS纬保留信息
	/*44*/_report_dykj_data[pos++] = 0;
	/*45*/_report_dykj_data[pos++] = 0;
	/*46*/_report_dykj_data[pos++] = 0;
	/*47*/_report_dykj_data[pos++] = 0;


	/*48*/_report_dykj_data[pos++] = 0;
	/*49*/_report_dykj_data[pos++] = 0;

	/*50*/_report_dykj_data[pos++] = _dykj_can_data._data_01[0];
	/*51*/_report_dykj_data[pos++] = _dykj_can_data._data_01[1];
	/*52*/_report_dykj_data[pos++] = _dykj_can_data._data_01[2];
	/*53*/_report_dykj_data[pos++] = _dykj_can_data._data_01[3];
	/*54*/_report_dykj_data[pos++] = _dykj_can_data._data_01[4];
	/*55*/_report_dykj_data[pos++] = _dykj_can_data._data_01[5];
	/*56*/_report_dykj_data[pos++] = _dykj_can_data._data_01[6];
	/*57*/_report_dykj_data[pos++] = _dykj_can_data._data_01[7];

	/*58*/_report_dykj_data[pos++] = _dykj_can_data._data_02[0];
	/*59*/_report_dykj_data[pos++] = _dykj_can_data._data_02[1];
	/*60*/_report_dykj_data[pos++] = _dykj_can_data._data_02[2];
	/*61*/_report_dykj_data[pos++] = _dykj_can_data._data_02[3];
	/*62*/_report_dykj_data[pos++] = _dykj_can_data._data_02[4];
	/*63*/_report_dykj_data[pos++] = _dykj_can_data._data_02[5];
	/*64*/_report_dykj_data[pos++] = _dykj_can_data._data_02[6];
	/*65*/_report_dykj_data[pos++] = _dykj_can_data._data_02[7];

	/*66*/_report_dykj_data[pos++] = _dykj_can_data._data_03[0];
	/*67*/_report_dykj_data[pos++] = _dykj_can_data._data_03[1];
	/*68*/_report_dykj_data[pos++] = _dykj_can_data._data_03[2];
	/*69*/_report_dykj_data[pos++] = _dykj_can_data._data_03[3];
	/*70*/_report_dykj_data[pos++] = _dykj_can_data._data_03[4];
	/*71*/_report_dykj_data[pos++] = _dykj_can_data._data_03[5];
	/*72*/_report_dykj_data[pos++] = _dykj_can_data._data_03[6];
	/*73*/_report_dykj_data[pos++] = _dykj_can_data._data_03[7];

	/*74*/_report_dykj_data[pos++] = _dykj_can_data._data_04[0];
	/*75*/_report_dykj_data[pos++] = _dykj_can_data._data_04[1];
	/*76*/_report_dykj_data[pos++] = _dykj_can_data._data_04[2];
	/*77*/_report_dykj_data[pos++] = _dykj_can_data._data_04[3];
	/*78*/_report_dykj_data[pos++] = _dykj_can_data._data_04[4];
	/*79*/_report_dykj_data[pos++] = _dykj_can_data._data_04[5];
	/*80*/_report_dykj_data[pos++] = _dykj_can_data._data_04[6];
	/*81*/_report_dykj_data[pos++] = _dykj_can_data._data_04[7];

	/*82*/_report_dykj_data[pos++] = _dykj_can_data._data_05[0];
	/*83*/_report_dykj_data[pos++] = _dykj_can_data._data_05[1];
	/*84*/_report_dykj_data[pos++] = _dykj_can_data._data_05[2];
	/*85*/_report_dykj_data[pos++] = _dykj_can_data._data_05[3];
	/*86*/_report_dykj_data[pos++] = _dykj_can_data._data_05[4];
	/*87*/_report_dykj_data[pos++] = _dykj_can_data._data_05[5];
	/*88*/_report_dykj_data[pos++] = _dykj_can_data._data_05[6];
	/*89*/_report_dykj_data[pos++] = _dykj_can_data._data_05[7];

	/*90*/_report_dykj_data[pos++] = _dykj_can_data._data_06[0];
	/*91*/_report_dykj_data[pos++] = _dykj_can_data._data_06[1];
	/*92*/_report_dykj_data[pos++] = _dykj_can_data._data_06[2];
	/*93*/_report_dykj_data[pos++] = _dykj_can_data._data_06[3];
	/*94*/_report_dykj_data[pos++] = _dykj_can_data._data_06[4];
	/*95*/_report_dykj_data[pos++] = _dykj_can_data._data_06[5];
	/*96*/_report_dykj_data[pos++] = _dykj_can_data._data_06[6];
	/*97*/_report_dykj_data[pos++] = _dykj_can_data._data_06[7];

	/*98*/_report_dykj_data[pos++] = _dykj_can_data._data_07[0];
	/*99*/_report_dykj_data[pos++] = _dykj_can_data._data_07[1];
	/*100*/_report_dykj_data[pos++] = _dykj_can_data._data_07[2];
	/*101*/_report_dykj_data[pos++] = _dykj_can_data._data_07[3];
	/*102*/_report_dykj_data[pos++] = _dykj_can_data._data_07[4];
	/*103*/_report_dykj_data[pos++] = _dykj_can_data._data_07[5];
	/*104*/_report_dykj_data[pos++] = _dykj_can_data._data_07[6];
	/*105*/_report_dykj_data[pos++] = _dykj_can_data._data_07[7];

	/*106*/_report_dykj_data[pos++] = _dykj_can_data._data_08[0];
	/*107*/_report_dykj_data[pos++] = _dykj_can_data._data_08[1];
	/*108*/_report_dykj_data[pos++] = _dykj_can_data._data_08[2];
	/*109*/_report_dykj_data[pos++] = _dykj_can_data._data_08[3];
	/*110*/_report_dykj_data[pos++] = _dykj_can_data._data_08[4];
	/*111*/_report_dykj_data[pos++] = _dykj_can_data._data_08[5];
	/*112*/_report_dykj_data[pos++] = _dykj_can_data._data_08[6];
	/*113*/_report_dykj_data[pos++] = _dykj_can_data._data_08[7];

	/*114*/_report_dykj_data[pos++] = _dykj_can_data._data_09[0];
	/*115*/_report_dykj_data[pos++] = _dykj_can_data._data_09[1];
	/*116*/_report_dykj_data[pos++] = _dykj_can_data._data_09[2];
	/*117*/_report_dykj_data[pos++] = _dykj_can_data._data_09[3];
	/*118*/_report_dykj_data[pos++] = _dykj_can_data._data_09[4];
	/*119*/_report_dykj_data[pos++] = _dykj_can_data._data_09[5];
	/*120*/_report_dykj_data[pos++] = _dykj_can_data._data_09[6];
	/*121*/_report_dykj_data[pos++] = _dykj_can_data._data_09[7];

	/*122*/_report_dykj_data[pos++] = _dykj_can_data._data_10[0];
	/*123*/_report_dykj_data[pos++] = _dykj_can_data._data_10[1];
	/*124*/_report_dykj_data[pos++] = _dykj_can_data._data_10[2];
	/*125*/_report_dykj_data[pos++] = _dykj_can_data._data_10[3];
	/*126*/_report_dykj_data[pos++] = _dykj_can_data._data_10[4];
	/*127*/_report_dykj_data[pos++] = _dykj_can_data._data_10[5];
	/*128*/_report_dykj_data[pos++] = _dykj_can_data._data_10[6];
	/*129*/_report_dykj_data[pos++] = _dykj_can_data._data_10[7];

	/*130*/_report_dykj_data[pos++] = 0;
	/*131*/_report_dykj_data[pos++] = 0;
	/*132*/_report_dykj_data[pos++] = 0;
	/*133*/_report_dykj_data[pos++] = 0;
	/*134*/_report_dykj_data[pos++] = 0;
	/*135*/_report_dykj_data[pos++] = 0;
	/*136*/_report_dykj_data[pos++] = 0;
	/*137*/_report_dykj_data[pos++] = 0;

	/*138*/_report_dykj_data[pos++] = 0;
	/*139*/_report_dykj_data[pos++] = 0;
	/*140*/_report_dykj_data[pos++] = 0;
	/*141*/_report_dykj_data[pos++] = 0;
	/*142*/_report_dykj_data[pos++] = 0;
	/*143*/_report_dykj_data[pos++] = 0;
	/*144*/_report_dykj_data[pos++] = 0;
	/*145*/_report_dykj_data[pos++] = 0;
}


static void _set_jsyt_frame(void)
{
	uint8_t shi, fen, miao;
	uint8_t pos = 0;
	//初始化
	memset(_report_jsyt_data, 0 ,sizeof(_report_jsyt_data));

	//车辆VIN - 24 字节
	pos = 0;
	/*01*/_report_jsyt_data[pos++] = 0x00;
	/*02*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x00];
	/*03*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x01];
	/*04*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x02];
	/*05*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x03];
	/*06*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x04];
	/*07*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x05];
	/*08*/_report_jsyt_data[pos++] = (_report_jsyt_data[0x00] + _report_jsyt_data[0x01] + _report_jsyt_data[0x02] + _report_jsyt_data[0x03] + _report_jsyt_data[0x04] + _report_jsyt_data[0x05] + _report_jsyt_data[0x06]) & 0xFF;
	
	/*09*/_report_jsyt_data[pos++] = 0x01;
	/*10*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x06];
	/*11*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x07];
	/*12*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x08];
	/*13*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x09];
	/*14*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x0A];
	/*15*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x0B];
	/*16*/_report_jsyt_data[pos++] = (_report_jsyt_data[0x08] + _report_jsyt_data[0x09] + _report_jsyt_data[0x0A] + _report_jsyt_data[0x0B] + _report_jsyt_data[0x0C] + _report_jsyt_data[0x0D] + _report_jsyt_data[0x0E]) & 0xFF;

	/*17*/_report_jsyt_data[pos++] = 0x02;
	/*18*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x0C];
	/*19*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x0D];
	/*20*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x0E];
	/*21*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x0F];
	/*22*/_report_jsyt_data[pos++] = g_car_config.car_vin[0x10];
	/*23*/_report_jsyt_data[pos++] = 0x00;
	/*24*/_report_jsyt_data[pos++] = (_report_jsyt_data[0x10] + _report_jsyt_data[0x11] + _report_jsyt_data[0x12] + _report_jsyt_data[0x13] + _report_jsyt_data[0x14] + _report_jsyt_data[0x15] + _report_jsyt_data[0x16]) & 0xFF;

	//记录采样时间
	/*25*/_report_jsyt_data[pos++] = g_ddmmyy % 100;//年
	/*26*/_report_jsyt_data[pos++] = (g_ddmmyy / 100) % 100;//月
	/*27*/_report_jsyt_data[pos++] = (g_ddmmyy / 10000) % 100;;//日
	shi = (g_hhmmss / 10000) + 8;
	shi = shi % 24;
	/*28*/_report_jsyt_data[pos++] = shi;//时
	fen = (g_hhmmss / 100) % 100;
	fen = fen % 60;
	/*29*/_report_jsyt_data[pos++] = fen;//分
	miao = g_hhmmss % 100;
	miao = miao % 60;
	/*30*/_report_jsyt_data[pos++] = miao;//秒

	debug_printf("%d %d %d %d %d %d\r\n", _report_jsyt_data[25],_report_jsyt_data[26],_report_jsyt_data[27],_report_jsyt_data[28],_report_jsyt_data[29],_report_jsyt_data[30]);

	//GPS定位状态
	/*31*/_report_jsyt_data[pos++] = g_status;//0000 0001

	//GPS经度信息
	/*32*/_report_jsyt_data[pos++] = (g_latitude & 0xFF000000) >> 24;
	/*33*/_report_jsyt_data[pos++] = (g_latitude & 0x00FF0000) >> 16;
	/*34*/_report_jsyt_data[pos++] = (g_latitude & 0x0000FF00) >> 8;
	/*35*/_report_jsyt_data[pos++] = (g_latitude & 0x000000FF);

	//GPS纬度信息
	/*36*/_report_jsyt_data[pos++] = (g_longitude & 0xFF000000) >> 24;
	/*37*/_report_jsyt_data[pos++] = (g_longitude & 0x00FF0000) >> 16;
	/*38*/_report_jsyt_data[pos++] = (g_longitude & 0x0000FF00) >> 8;
	/*39*/_report_jsyt_data[pos++] = (g_longitude & 0x000000FF);

	//GPS速度
	/*41*/_report_jsyt_data[pos++] = (g_speed & 0xFF00) >> 8;
	/*40*/_report_jsyt_data[pos++] = (g_speed & 0x00FF);

	//GPS方向
	/*43*/_report_jsyt_data[pos++] = (g_course & 0xFF00) >> 8;
	/*42*/_report_jsyt_data[pos++] = (g_course & 0x00FF);

	//GPS纬保留信息
	/*44*/_report_jsyt_data[pos++] = 0;
	/*45*/_report_jsyt_data[pos++] = 0;
	/*46*/_report_jsyt_data[pos++] = 0;
	/*47*/_report_jsyt_data[pos++] = 0;


	/*48*/_report_jsyt_data[pos++] = 0;
	/*49*/_report_jsyt_data[pos++] = 0;

	/*50*/_report_jsyt_data[pos++] = _jsyt_can_data._data_01[0];
	/*51*/_report_jsyt_data[pos++] = _jsyt_can_data._data_01[1];
	/*52*/_report_jsyt_data[pos++] = _jsyt_can_data._data_01[2];
	/*53*/_report_jsyt_data[pos++] = _jsyt_can_data._data_01[3];
	/*54*/_report_jsyt_data[pos++] = _jsyt_can_data._data_01[4];
	/*55*/_report_jsyt_data[pos++] = _jsyt_can_data._data_01[5];
	/*56*/_report_jsyt_data[pos++] = _jsyt_can_data._data_01[6];
	/*57*/_report_jsyt_data[pos++] = _jsyt_can_data._data_01[7];

	/*58*/_report_jsyt_data[pos++] = _jsyt_can_data._data_02[0];
	/*59*/_report_jsyt_data[pos++] = _jsyt_can_data._data_02[1];
	/*60*/_report_jsyt_data[pos++] = _jsyt_can_data._data_02[2];
	/*61*/_report_jsyt_data[pos++] = _jsyt_can_data._data_02[3];
	/*62*/_report_jsyt_data[pos++] = _jsyt_can_data._data_02[4];
	/*63*/_report_jsyt_data[pos++] = _jsyt_can_data._data_02[5];
	/*64*/_report_jsyt_data[pos++] = _jsyt_can_data._data_02[6];
	/*65*/_report_jsyt_data[pos++] = _jsyt_can_data._data_02[7];

	/*66*/_report_jsyt_data[pos++] = _jsyt_can_data._data_03[0];
	/*67*/_report_jsyt_data[pos++] = _jsyt_can_data._data_03[1];
	/*68*/_report_jsyt_data[pos++] = _jsyt_can_data._data_03[2];
	/*69*/_report_jsyt_data[pos++] = _jsyt_can_data._data_03[3];
	/*70*/_report_jsyt_data[pos++] = _jsyt_can_data._data_03[4];
	/*71*/_report_jsyt_data[pos++] = _jsyt_can_data._data_03[5];
	/*72*/_report_jsyt_data[pos++] = _jsyt_can_data._data_03[6];
	/*73*/_report_jsyt_data[pos++] = _jsyt_can_data._data_03[7];

	/*74*/_report_jsyt_data[pos++] = _jsyt_can_data._data_04[0];
	/*75*/_report_jsyt_data[pos++] = _jsyt_can_data._data_04[1];
	/*76*/_report_jsyt_data[pos++] = _jsyt_can_data._data_04[2];
	/*77*/_report_jsyt_data[pos++] = _jsyt_can_data._data_04[3];
	/*78*/_report_jsyt_data[pos++] = _jsyt_can_data._data_04[4];
	/*79*/_report_jsyt_data[pos++] = _jsyt_can_data._data_04[5];
	/*80*/_report_jsyt_data[pos++] = _jsyt_can_data._data_04[6];
	/*81*/_report_jsyt_data[pos++] = _jsyt_can_data._data_04[7];

	/*82*/_report_jsyt_data[pos++] = _jsyt_can_data._data_05[0];
	/*83*/_report_jsyt_data[pos++] = _jsyt_can_data._data_05[1];
	/*84*/_report_jsyt_data[pos++] = _jsyt_can_data._data_05[2];
	/*85*/_report_jsyt_data[pos++] = _jsyt_can_data._data_05[3];
	/*86*/_report_jsyt_data[pos++] = _jsyt_can_data._data_05[4];
	/*87*/_report_jsyt_data[pos++] = _jsyt_can_data._data_05[5];
	/*88*/_report_jsyt_data[pos++] = _jsyt_can_data._data_05[6];
	/*89*/_report_jsyt_data[pos++] = _jsyt_can_data._data_05[7];

	/*90*/_report_jsyt_data[pos++] = _jsyt_can_data._data_06[0];
	/*91*/_report_jsyt_data[pos++] = _jsyt_can_data._data_06[1];
	/*92*/_report_jsyt_data[pos++] = _jsyt_can_data._data_06[2];
	/*93*/_report_jsyt_data[pos++] = _jsyt_can_data._data_06[3];
	/*94*/_report_jsyt_data[pos++] = _jsyt_can_data._data_06[4];
	/*95*/_report_jsyt_data[pos++] = _jsyt_can_data._data_06[5];
	/*96*/_report_jsyt_data[pos++] = _jsyt_can_data._data_06[6];
	/*97*/_report_jsyt_data[pos++] = _jsyt_can_data._data_06[7];

	/*98*/_report_jsyt_data[pos++] = _jsyt_can_data._data_07[0];
	/*99*/_report_jsyt_data[pos++] = _jsyt_can_data._data_07[1];
	/*100*/_report_jsyt_data[pos++] = _jsyt_can_data._data_07[2];
	/*101*/_report_jsyt_data[pos++] = _jsyt_can_data._data_07[3];
	/*102*/_report_jsyt_data[pos++] = _jsyt_can_data._data_07[4];
	/*103*/_report_jsyt_data[pos++] = _jsyt_can_data._data_07[5];
	/*104*/_report_jsyt_data[pos++] = _jsyt_can_data._data_07[6];
	/*105*/_report_jsyt_data[pos++] = _jsyt_can_data._data_07[7];

	/*106*/_report_jsyt_data[pos++] = _jsyt_can_data._data_08[0];
	/*107*/_report_jsyt_data[pos++] = _jsyt_can_data._data_08[1];
	/*108*/_report_jsyt_data[pos++] = _jsyt_can_data._data_08[2];
	/*109*/_report_jsyt_data[pos++] = _jsyt_can_data._data_08[3];
	/*110*/_report_jsyt_data[pos++] = _jsyt_can_data._data_08[4];
	/*111*/_report_jsyt_data[pos++] = _jsyt_can_data._data_08[5];
	/*112*/_report_jsyt_data[pos++] = _jsyt_can_data._data_08[6];
	/*113*/_report_jsyt_data[pos++] = _jsyt_can_data._data_08[7];

	/*114*/_report_jsyt_data[pos++] = _jsyt_can_data._data_09[0];
	/*115*/_report_jsyt_data[pos++] = _jsyt_can_data._data_09[1];
	/*116*/_report_jsyt_data[pos++] = _jsyt_can_data._data_09[2];
	/*117*/_report_jsyt_data[pos++] = _jsyt_can_data._data_09[3];
	/*118*/_report_jsyt_data[pos++] = _jsyt_can_data._data_09[4];
	/*119*/_report_jsyt_data[pos++] = _jsyt_can_data._data_09[5];
	/*120*/_report_jsyt_data[pos++] = _jsyt_can_data._data_09[6];
	/*121*/_report_jsyt_data[pos++] = _jsyt_can_data._data_09[7];

	/*122*/_report_jsyt_data[pos++] = _jsyt_can_data._data_10[0];
	/*123*/_report_jsyt_data[pos++] = _jsyt_can_data._data_10[1];
	/*124*/_report_jsyt_data[pos++] = _jsyt_can_data._data_10[2];
	/*125*/_report_jsyt_data[pos++] = _jsyt_can_data._data_10[3];
	/*126*/_report_jsyt_data[pos++] = _jsyt_can_data._data_10[4];
	/*127*/_report_jsyt_data[pos++] = _jsyt_can_data._data_10[5];
	/*128*/_report_jsyt_data[pos++] = _jsyt_can_data._data_10[6];
	/*129*/_report_jsyt_data[pos++] = _jsyt_can_data._data_10[7];

	/*130*/_report_jsyt_data[pos++] = 0;
	/*131*/_report_jsyt_data[pos++] = 0;
	/*132*/_report_jsyt_data[pos++] = 0;
	/*133*/_report_jsyt_data[pos++] = 0;
	/*134*/_report_jsyt_data[pos++] = 0;
	/*135*/_report_jsyt_data[pos++] = 0;
	/*136*/_report_jsyt_data[pos++] = 0;
	/*137*/_report_jsyt_data[pos++] = 0;

	/*138*/_report_jsyt_data[pos++] = 0;
	/*139*/_report_jsyt_data[pos++] = 0;
	/*140*/_report_jsyt_data[pos++] = 0;
	/*141*/_report_jsyt_data[pos++] = 0;
	/*142*/_report_jsyt_data[pos++] = 0;
	/*143*/_report_jsyt_data[pos++] = 0;
	/*144*/_report_jsyt_data[pos++] = 0;
	/*145*/_report_jsyt_data[pos++] = 0;
}

void _set_vict_frame()
{
	uint8_t shi, fen, miao;
	uint8_t pos = 0;
	//初始化
	memset(_report_vict_data, 0 ,sizeof(_report_vict_data));

	//车辆VIN - 24 字节
	pos = 0;
	/*01*/_report_vict_data[pos++] = 0x00;
	/*02*/_report_vict_data[pos++] = g_car_config.car_vin[0x00];
	/*03*/_report_vict_data[pos++] = g_car_config.car_vin[0x01];
	/*04*/_report_vict_data[pos++] = g_car_config.car_vin[0x02];
	/*05*/_report_vict_data[pos++] = g_car_config.car_vin[0x03];
	/*06*/_report_vict_data[pos++] = g_car_config.car_vin[0x04];
	/*07*/_report_vict_data[pos++] = g_car_config.car_vin[0x05];
	/*08*/_report_vict_data[pos++] = (_report_vict_data[0x00] + _report_vict_data[0x01] + _report_vict_data[0x02] + _report_vict_data[0x03] + _report_vict_data[0x04] + _report_vict_data[0x05] + _report_vict_data[0x06]) & 0xFF;
	
	/*09*/_report_vict_data[pos++] = 0x01;
	/*10*/_report_vict_data[pos++] = g_car_config.car_vin[0x06];
	/*11*/_report_vict_data[pos++] = g_car_config.car_vin[0x07];
	/*12*/_report_vict_data[pos++] = g_car_config.car_vin[0x08];
	/*13*/_report_vict_data[pos++] = g_car_config.car_vin[0x09];
	/*14*/_report_vict_data[pos++] = g_car_config.car_vin[0x0A];
	/*15*/_report_vict_data[pos++] = g_car_config.car_vin[0x0B];
	/*16*/_report_vict_data[pos++] = (_report_vict_data[0x08] + _report_vict_data[0x09] + _report_vict_data[0x0A] + _report_vict_data[0x0B] + _report_vict_data[0x0C] + _report_vict_data[0x0D] + _report_vict_data[0x0E]) & 0xFF;

	/*17*/_report_vict_data[pos++] = 0x02;
	/*18*/_report_vict_data[pos++] = g_car_config.car_vin[0x0C];
	/*19*/_report_vict_data[pos++] = g_car_config.car_vin[0x0D];
	/*20*/_report_vict_data[pos++] = g_car_config.car_vin[0x0E];
	/*21*/_report_vict_data[pos++] = g_car_config.car_vin[0x0F];
	/*22*/_report_vict_data[pos++] = g_car_config.car_vin[0x10];
	/*23*/_report_vict_data[pos++] = 0x00;
	/*24*/_report_vict_data[pos++] = (_report_vict_data[0x10] + _report_vict_data[0x11] + _report_vict_data[0x12] + _report_vict_data[0x13] + _report_vict_data[0x14] + _report_vict_data[0x15] + _report_vict_data[0x16]) & 0xFF;

	//记录采样时间
	/*25*/_report_vict_data[pos++] = g_ddmmyy % 100;//年
	/*26*/_report_vict_data[pos++] = (g_ddmmyy / 100) % 100;//月
	/*27*/_report_vict_data[pos++] = (g_ddmmyy / 10000) % 100;;//日
	shi = (g_hhmmss / 10000) + 8;
	shi = shi % 24;
	/*28*/_report_vict_data[pos++] = shi;//时
	fen = (g_hhmmss / 100) % 100;
	fen = fen % 60;
	/*29*/_report_vict_data[pos++] = fen;//分
	miao = g_hhmmss % 100;
	miao = miao % 60;
	/*30*/_report_vict_data[pos++] = miao;//秒

	debug_printf("%d %d %d %d %d %d\r\n", _report_vict_data[25],_report_vict_data[26],_report_vict_data[27],_report_vict_data[28],_report_vict_data[29],_report_vict_data[30]);

	//GPS定位状态
	/*31*/_report_vict_data[pos++] = g_status;//0000 0001

	//GPS经度信息
	/*32*/_report_vict_data[pos++] = (g_latitude & 0xFF000000) >> 24;
	/*33*/_report_vict_data[pos++] = (g_latitude & 0x00FF0000) >> 16;
	/*34*/_report_vict_data[pos++] = (g_latitude & 0x0000FF00) >> 8;
	/*35*/_report_vict_data[pos++] = (g_latitude & 0x000000FF);

	//GPS纬度信息
	/*36*/_report_vict_data[pos++] = (g_longitude & 0xFF000000) >> 24;
	/*37*/_report_vict_data[pos++] = (g_longitude & 0x00FF0000) >> 16;
	/*38*/_report_vict_data[pos++] = (g_longitude & 0x0000FF00) >> 8;
	/*39*/_report_vict_data[pos++] = (g_longitude & 0x000000FF);

	//GPS速度
	/*41*/_report_vict_data[pos++] = (g_speed & 0xFF00) >> 8;
	/*40*/_report_vict_data[pos++] = (g_speed & 0x00FF);

	//GPS方向
	/*43*/_report_vict_data[pos++] = (g_course & 0xFF00) >> 8;
	/*42*/_report_vict_data[pos++] = (g_course & 0x00FF);

	//GPS纬保留信息
	/*44*/_report_vict_data[pos++] = 0;
	/*45*/_report_vict_data[pos++] = 0;
	/*46*/_report_vict_data[pos++] = 0;
	/*47*/_report_vict_data[pos++] = 0;

	/*48*/_report_vict_data[pos++] = 'V';
	/*49*/_report_vict_data[pos++] = 'T';

	/*50*/_report_vict_data[pos++] = _vict_can_data._data_01[0];
	/*51*/_report_vict_data[pos++] = _vict_can_data._data_01[1];
	/*52*/_report_vict_data[pos++] = _vict_can_data._data_01[2];
	/*53*/_report_vict_data[pos++] = _vict_can_data._data_01[3];
	/*54*/_report_vict_data[pos++] = _vict_can_data._data_01[4];
	/*55*/_report_vict_data[pos++] = _vict_can_data._data_01[5];
	/*56*/_report_vict_data[pos++] = _vict_can_data._data_01[6];
	/*57*/_report_vict_data[pos++] = _vict_can_data._data_01[7];

	/*58*/_report_vict_data[pos++] = _vict_can_data._data_02[0];
	/*59*/_report_vict_data[pos++] = _vict_can_data._data_02[1];
	/*60*/_report_vict_data[pos++] = _vict_can_data._data_02[2];
	/*61*/_report_vict_data[pos++] = _vict_can_data._data_02[3];
	/*62*/_report_vict_data[pos++] = _vict_can_data._data_02[4];
	/*63*/_report_vict_data[pos++] = _vict_can_data._data_02[5];
	/*64*/_report_vict_data[pos++] = _vict_can_data._data_02[6];
	/*65*/_report_vict_data[pos++] = _vict_can_data._data_02[7];

	/*66*/_report_vict_data[pos++] = _vict_can_data._data_03[0];
	/*67*/_report_vict_data[pos++] = _vict_can_data._data_03[1];
	/*68*/_report_vict_data[pos++] = _vict_can_data._data_03[2];
	/*69*/_report_vict_data[pos++] = _vict_can_data._data_03[3];
	/*70*/_report_vict_data[pos++] = _vict_can_data._data_03[4];
	/*71*/_report_vict_data[pos++] = _vict_can_data._data_03[5];
	/*72*/_report_vict_data[pos++] = _vict_can_data._data_03[6];
	/*73*/_report_vict_data[pos++] = _vict_can_data._data_03[7];

	/*74*/_report_vict_data[pos++] = _vict_can_data._data_04[0];
	/*75*/_report_vict_data[pos++] = _vict_can_data._data_04[1];
	/*76*/_report_vict_data[pos++] = _vict_can_data._data_04[2];
	/*77*/_report_vict_data[pos++] = _vict_can_data._data_04[3];
	/*78*/_report_vict_data[pos++] = _vict_can_data._data_04[4];
	/*79*/_report_vict_data[pos++] = _vict_can_data._data_04[5];
	/*80*/_report_vict_data[pos++] = _vict_can_data._data_04[6];
	/*81*/_report_vict_data[pos++] = _vict_can_data._data_04[7];

	/*82*/_report_vict_data[pos++] = _vict_can_data._data_05[0];
	/*83*/_report_vict_data[pos++] = _vict_can_data._data_05[1];
	/*84*/_report_vict_data[pos++] = _vict_can_data._data_05[2];
	/*85*/_report_vict_data[pos++] = _vict_can_data._data_05[3];
	/*86*/_report_vict_data[pos++] = _vict_can_data._data_05[4];
	/*87*/_report_vict_data[pos++] = _vict_can_data._data_05[5];
	/*88*/_report_vict_data[pos++] = _vict_can_data._data_05[6];
	/*89*/_report_vict_data[pos++] = _vict_can_data._data_05[7];

	/*90*/_report_vict_data[pos++] = _vict_can_data._data_06[0];
	/*91*/_report_vict_data[pos++] = _vict_can_data._data_06[1];
	/*92*/_report_vict_data[pos++] = _vict_can_data._data_06[2];
	/*93*/_report_vict_data[pos++] = _vict_can_data._data_06[3];
	/*94*/_report_vict_data[pos++] = _vict_can_data._data_06[4];
	/*95*/_report_vict_data[pos++] = _vict_can_data._data_06[5];
	/*96*/_report_vict_data[pos++] = _vict_can_data._data_06[6];
	/*97*/_report_vict_data[pos++] = _vict_can_data._data_06[7];

	/*98*/_report_vict_data[pos++] = _vict_can_data._data_07[0];
	/*99*/_report_vict_data[pos++] = _vict_can_data._data_07[1];
	/*100*/_report_vict_data[pos++] = _vict_can_data._data_07[2];
	/*101*/_report_vict_data[pos++] = _vict_can_data._data_07[3];
	/*102*/_report_vict_data[pos++] = _vict_can_data._data_07[4];
	/*103*/_report_vict_data[pos++] = _vict_can_data._data_07[5];
	/*104*/_report_vict_data[pos++] = _vict_can_data._data_07[6];
	/*105*/_report_vict_data[pos++] = _vict_can_data._data_07[7];

	/*106*/_report_vict_data[pos++] = _vict_can_data._data_08[0];
	/*107*/_report_vict_data[pos++] = _vict_can_data._data_08[1];
	/*108*/_report_vict_data[pos++] = _vict_can_data._data_08[2];
	/*109*/_report_vict_data[pos++] = _vict_can_data._data_08[3];
	/*110*/_report_vict_data[pos++] = _vict_can_data._data_08[4];
	/*111*/_report_vict_data[pos++] = _vict_can_data._data_08[5];
	/*112*/_report_vict_data[pos++] = _vict_can_data._data_08[6];
	/*113*/_report_vict_data[pos++] = _vict_can_data._data_08[7];

	/*114*/_report_vict_data[pos++] = _vict_can_data._data_09[0];
	/*115*/_report_vict_data[pos++] = _vict_can_data._data_09[1];
	/*116*/_report_vict_data[pos++] = _vict_can_data._data_09[2];
	/*117*/_report_vict_data[pos++] = _vict_can_data._data_09[3];
	/*118*/_report_vict_data[pos++] = _vict_can_data._data_09[4];
	/*119*/_report_vict_data[pos++] = _vict_can_data._data_09[5];
	/*120*/_report_vict_data[pos++] = _vict_can_data._data_09[6];
	/*121*/_report_vict_data[pos++] = _vict_can_data._data_09[7];

	/*122*/_report_vict_data[pos++] = _vict_can_data._data_10[0];
	/*123*/_report_vict_data[pos++] = _vict_can_data._data_10[1];
	/*124*/_report_vict_data[pos++] = _vict_can_data._data_10[2];
	/*125*/_report_vict_data[pos++] = _vict_can_data._data_10[3];
	/*126*/_report_vict_data[pos++] = _vict_can_data._data_10[4];
	/*127*/_report_vict_data[pos++] = _vict_can_data._data_10[5];
	/*128*/_report_vict_data[pos++] = _vict_can_data._data_10[6];
	/*129*/_report_vict_data[pos++] = _vict_can_data._data_10[7];

	/*130*/_report_vict_data[pos++] = _vict_can_data._data_11[0];
	/*131*/_report_vict_data[pos++] = _vict_can_data._data_11[1];
	/*132*/_report_vict_data[pos++] = _vict_can_data._data_11[2];
	/*133*/_report_vict_data[pos++] = _vict_can_data._data_11[3];
	/*134*/_report_vict_data[pos++] = _vict_can_data._data_11[4];
	/*135*/_report_vict_data[pos++] = _vict_can_data._data_11[5];
	/*136*/_report_vict_data[pos++] = _vict_can_data._data_11[6];
	/*137*/_report_vict_data[pos++] = _vict_can_data._data_11[7];

	/*138*/_report_vict_data[pos++] = _vict_can_data._data_12[0];
	/*139*/_report_vict_data[pos++] = _vict_can_data._data_12[1];
	/*140*/_report_vict_data[pos++] = _vict_can_data._data_12[2];
	/*141*/_report_vict_data[pos++] = _vict_can_data._data_12[3];
	/*142*/_report_vict_data[pos++] = _vict_can_data._data_12[4];
	/*143*/_report_vict_data[pos++] = _vict_can_data._data_12[5];
	/*144*/_report_vict_data[pos++] = _vict_can_data._data_12[6];
	/*145*/_report_vict_data[pos++] = _vict_can_data._data_12[7];

	/*146*/_report_vict_data[pos++] = _vict_can_data._data_13[0];
	/*147*/_report_vict_data[pos++] = _vict_can_data._data_13[1];
	/*148*/_report_vict_data[pos++] = _vict_can_data._data_13[2];
	/*149*/_report_vict_data[pos++] = _vict_can_data._data_13[3];
	/*150*/_report_vict_data[pos++] = _vict_can_data._data_13[4];
	/*151*/_report_vict_data[pos++] = _vict_can_data._data_13[5];
	/*152*/_report_vict_data[pos++] = _vict_can_data._data_13[6];
	/*153*/_report_vict_data[pos++] = _vict_can_data._data_13[7];

	/*154*/_report_vict_data[pos++] = _vict_can_data._data_14[0];
	/*155*/_report_vict_data[pos++] = _vict_can_data._data_14[1];
	/*156*/_report_vict_data[pos++] = _vict_can_data._data_14[2];
	/*157*/_report_vict_data[pos++] = _vict_can_data._data_14[3];
	/*158*/_report_vict_data[pos++] = _vict_can_data._data_14[4];
	/*159*/_report_vict_data[pos++] = _vict_can_data._data_14[5];
	/*160*/_report_vict_data[pos++] = _vict_can_data._data_14[6];
	/*161*/_report_vict_data[pos++] = _vict_can_data._data_14[7];

	/*162*/_report_vict_data[pos++] = _vict_can_data._data_15[0];
	/*163*/_report_vict_data[pos++] = _vict_can_data._data_15[1];
	/*164*/_report_vict_data[pos++] = _vict_can_data._data_15[2];
	/*165*/_report_vict_data[pos++] = _vict_can_data._data_15[3];
	/*166*/_report_vict_data[pos++] = _vict_can_data._data_15[4];
	/*167*/_report_vict_data[pos++] = _vict_can_data._data_15[5];
	/*168*/_report_vict_data[pos++] = _vict_can_data._data_15[6];
	/*169*/_report_vict_data[pos++] = _vict_can_data._data_15[7];

	/*170*/_report_vict_data[pos++] = _vict_can_data._data_16[0];
	/*171*/_report_vict_data[pos++] = _vict_can_data._data_16[1];
	/*172*/_report_vict_data[pos++] = _vict_can_data._data_16[2];
	/*173*/_report_vict_data[pos++] = _vict_can_data._data_16[3];
	/*174*/_report_vict_data[pos++] = _vict_can_data._data_16[4];
	/*175*/_report_vict_data[pos++] = _vict_can_data._data_16[5];
	/*176*/_report_vict_data[pos++] = _vict_can_data._data_16[6];
	/*177*/_report_vict_data[pos++] = _vict_can_data._data_16[7];

	/*178*/_report_vict_data[pos++] = 0;
	/*179*/_report_vict_data[pos++] = 0;
	/*180*/_report_vict_data[pos++] = 0;
	/*181*/_report_vict_data[pos++] = 0;
	/*182*/_report_vict_data[pos++] = 0;
	/*183*/_report_vict_data[pos++] = 0;
	/*184*/_report_vict_data[pos++] = 0;
	/*185*/_report_vict_data[pos++] = 0;
}


void _set_ydzy_frame()
{
	uint8_t shi, fen, miao;
	uint8_t pos = 0;
	//初始化
	memset(_report_ydzy_data, 0 ,sizeof(_report_ydzy_data));

	//车辆VIN - 24 字节
	pos = 0;
	/*01*/_report_ydzy_data[pos++] = 0x00;
	/*02*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x00];
	/*03*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x01];
	/*04*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x02];
	/*05*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x03];
	/*06*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x04];
	/*07*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x05];
	/*08*/_report_ydzy_data[pos++] = (_report_ydzy_data[0x00] + _report_ydzy_data[0x01] + _report_ydzy_data[0x02] + _report_ydzy_data[0x03] + _report_ydzy_data[0x04] + _report_ydzy_data[0x05] + _report_ydzy_data[0x06]) & 0xFF;
	
	/*09*/_report_ydzy_data[pos++] = 0x01;
	/*10*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x06];
	/*11*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x07];
	/*12*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x08];
	/*13*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x09];
	/*14*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x0A];
	/*15*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x0B];
	/*16*/_report_ydzy_data[pos++] = (_report_ydzy_data[0x08] + _report_ydzy_data[0x09] + _report_ydzy_data[0x0A] + _report_ydzy_data[0x0B] + _report_ydzy_data[0x0C] + _report_ydzy_data[0x0D] + _report_ydzy_data[0x0E]) & 0xFF;

	/*17*/_report_ydzy_data[pos++] = 0x02;
	/*18*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x0C];
	/*19*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x0D];
	/*20*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x0E];
	/*21*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x0F];
	/*22*/_report_ydzy_data[pos++] = g_car_config.car_vin[0x10];
	/*23*/_report_ydzy_data[pos++] = 0x00;
	/*24*/_report_ydzy_data[pos++] = (_report_ydzy_data[0x10] + _report_ydzy_data[0x11] + _report_ydzy_data[0x12] + _report_ydzy_data[0x13] + _report_ydzy_data[0x14] + _report_ydzy_data[0x15] + _report_ydzy_data[0x16]) & 0xFF;

	//记录采样时间
	/*25*/_report_ydzy_data[pos++] = g_ddmmyy % 100;//年
	/*26*/_report_ydzy_data[pos++] = (g_ddmmyy / 100) % 100;//月
	/*27*/_report_ydzy_data[pos++] = (g_ddmmyy / 10000) % 100;;//日
	shi = (g_hhmmss / 10000) + 8;
	shi = shi % 24;
	/*28*/_report_ydzy_data[pos++] = shi;//时
	fen = (g_hhmmss / 100) % 100;
	fen = fen % 60;
	/*29*/_report_ydzy_data[pos++] = fen;//分
	miao = g_hhmmss % 100;
	miao = miao % 60;
	/*30*/_report_ydzy_data[pos++] = miao;//秒

	debug_printf("%d %d %d %d %d %d\r\n", _report_ydzy_data[25],_report_ydzy_data[26],_report_ydzy_data[27],_report_ydzy_data[28],_report_ydzy_data[29],_report_ydzy_data[30]);

	//GPS定位状态
	/*31*/_report_ydzy_data[pos++] = g_status;//0000 0001

	//GPS经度信息
	/*32*/_report_ydzy_data[pos++] = (g_latitude & 0xFF000000) >> 24;
	/*33*/_report_ydzy_data[pos++] = (g_latitude & 0x00FF0000) >> 16;
	/*34*/_report_ydzy_data[pos++] = (g_latitude & 0x0000FF00) >> 8;
	/*35*/_report_ydzy_data[pos++] = (g_latitude & 0x000000FF);

	//GPS纬度信息
	/*36*/_report_ydzy_data[pos++] = (g_longitude & 0xFF000000) >> 24;
	/*37*/_report_ydzy_data[pos++] = (g_longitude & 0x00FF0000) >> 16;
	/*38*/_report_ydzy_data[pos++] = (g_longitude & 0x0000FF00) >> 8;
	/*39*/_report_ydzy_data[pos++] = (g_longitude & 0x000000FF);

	//GPS速度
	/*41*/_report_ydzy_data[pos++] = (g_speed & 0xFF00) >> 8;
	/*40*/_report_ydzy_data[pos++] = (g_speed & 0x00FF);

	//GPS方向
	/*43*/_report_ydzy_data[pos++] = (g_course & 0xFF00) >> 8;
	/*42*/_report_ydzy_data[pos++] = (g_course & 0x00FF);

	//GPS纬保留信息
	/*44*/_report_ydzy_data[pos++] = 0;
	/*45*/_report_ydzy_data[pos++] = 0;
	/*46*/_report_ydzy_data[pos++] = 0;
	/*47*/_report_ydzy_data[pos++] = 0;

	/*48*/_report_ydzy_data[pos++] = 'Y';
	/*49*/_report_ydzy_data[pos++] = 'Z';

	/*50*/_report_ydzy_data[pos++] = _ydzy_can_data._data_01[0];
	/*51*/_report_ydzy_data[pos++] = _ydzy_can_data._data_01[1];
	/*52*/_report_ydzy_data[pos++] = _ydzy_can_data._data_01[2];
	/*53*/_report_ydzy_data[pos++] = _ydzy_can_data._data_01[3];
	/*54*/_report_ydzy_data[pos++] = _ydzy_can_data._data_01[4];
	/*55*/_report_ydzy_data[pos++] = _ydzy_can_data._data_01[5];
	/*56*/_report_ydzy_data[pos++] = _ydzy_can_data._data_01[6];
	/*57*/_report_ydzy_data[pos++] = _ydzy_can_data._data_01[7];

	/*58*/_report_ydzy_data[pos++] = _ydzy_can_data._data_02[0];
	/*59*/_report_ydzy_data[pos++] = _ydzy_can_data._data_02[1];
	/*60*/_report_ydzy_data[pos++] = _ydzy_can_data._data_02[2];
	/*61*/_report_ydzy_data[pos++] = _ydzy_can_data._data_02[3];
	/*62*/_report_ydzy_data[pos++] = _ydzy_can_data._data_02[4];
	/*63*/_report_ydzy_data[pos++] = _ydzy_can_data._data_02[5];
	/*64*/_report_ydzy_data[pos++] = _ydzy_can_data._data_02[6];
	/*65*/_report_ydzy_data[pos++] = _ydzy_can_data._data_02[7];

	/*66*/_report_ydzy_data[pos++] = _ydzy_can_data._data_03[0];
	/*67*/_report_ydzy_data[pos++] = _ydzy_can_data._data_03[1];
	/*68*/_report_ydzy_data[pos++] = _ydzy_can_data._data_03[2];
	/*69*/_report_ydzy_data[pos++] = _ydzy_can_data._data_03[3];
	/*70*/_report_ydzy_data[pos++] = _ydzy_can_data._data_03[4];
	/*71*/_report_ydzy_data[pos++] = _ydzy_can_data._data_03[5];
	/*72*/_report_ydzy_data[pos++] = _ydzy_can_data._data_03[6];
	/*73*/_report_ydzy_data[pos++] = _ydzy_can_data._data_03[7];

	/*74*/_report_ydzy_data[pos++] = _ydzy_can_data._data_04[0];
	/*75*/_report_ydzy_data[pos++] = _ydzy_can_data._data_04[1];
	/*76*/_report_ydzy_data[pos++] = _ydzy_can_data._data_04[2];
	/*77*/_report_ydzy_data[pos++] = _ydzy_can_data._data_04[3];
	/*78*/_report_ydzy_data[pos++] = _ydzy_can_data._data_04[4];
	/*79*/_report_ydzy_data[pos++] = _ydzy_can_data._data_04[5];
	/*80*/_report_ydzy_data[pos++] = _ydzy_can_data._data_04[6];
	/*81*/_report_ydzy_data[pos++] = _ydzy_can_data._data_04[7];

	/*82*/_report_ydzy_data[pos++] = _ydzy_can_data._data_05[0];
	/*83*/_report_ydzy_data[pos++] = _ydzy_can_data._data_05[1];
	/*84*/_report_ydzy_data[pos++] = _ydzy_can_data._data_05[2];
	/*85*/_report_ydzy_data[pos++] = _ydzy_can_data._data_05[3];
	/*86*/_report_ydzy_data[pos++] = _ydzy_can_data._data_05[4];
	/*87*/_report_ydzy_data[pos++] = _ydzy_can_data._data_05[5];
	/*88*/_report_ydzy_data[pos++] = _ydzy_can_data._data_05[6];
	/*89*/_report_ydzy_data[pos++] = _ydzy_can_data._data_05[7];

	/*90*/_report_ydzy_data[pos++] = _ydzy_can_data._data_06[0];
	/*91*/_report_ydzy_data[pos++] = _ydzy_can_data._data_06[1];
	/*92*/_report_ydzy_data[pos++] = _ydzy_can_data._data_06[2];
	/*93*/_report_ydzy_data[pos++] = _ydzy_can_data._data_06[3];
	/*94*/_report_ydzy_data[pos++] = _ydzy_can_data._data_06[4];
	/*95*/_report_ydzy_data[pos++] = _ydzy_can_data._data_06[5];
	/*96*/_report_ydzy_data[pos++] = _ydzy_can_data._data_06[6];
	/*97*/_report_ydzy_data[pos++] = _ydzy_can_data._data_06[7];

	/*98*/_report_ydzy_data[pos++] = _ydzy_can_data._data_07[0];
	/*99*/_report_ydzy_data[pos++] = _ydzy_can_data._data_07[1];
	/*100*/_report_ydzy_data[pos++] = _ydzy_can_data._data_07[2];
	/*101*/_report_ydzy_data[pos++] = _ydzy_can_data._data_07[3];
	/*102*/_report_ydzy_data[pos++] = _ydzy_can_data._data_07[4];
	/*103*/_report_ydzy_data[pos++] = _ydzy_can_data._data_07[5];
	/*104*/_report_ydzy_data[pos++] = _ydzy_can_data._data_07[6];
	/*105*/_report_ydzy_data[pos++] = _ydzy_can_data._data_07[7];

	/*106*/_report_ydzy_data[pos++] = _ydzy_can_data._data_08[0];
	/*107*/_report_ydzy_data[pos++] = _ydzy_can_data._data_08[1];
	/*108*/_report_ydzy_data[pos++] = _ydzy_can_data._data_08[2];
	/*109*/_report_ydzy_data[pos++] = _ydzy_can_data._data_08[3];
	/*110*/_report_ydzy_data[pos++] = _ydzy_can_data._data_08[4];
	/*111*/_report_ydzy_data[pos++] = _ydzy_can_data._data_08[5];
	/*112*/_report_ydzy_data[pos++] = _ydzy_can_data._data_08[6];
	/*113*/_report_ydzy_data[pos++] = _ydzy_can_data._data_08[7];

	/*114*/_report_ydzy_data[pos++] = _ydzy_can_data._data_09[0];
	/*115*/_report_ydzy_data[pos++] = _ydzy_can_data._data_09[1];
	/*116*/_report_ydzy_data[pos++] = _ydzy_can_data._data_09[2];
	/*117*/_report_ydzy_data[pos++] = _ydzy_can_data._data_09[3];
	/*118*/_report_ydzy_data[pos++] = _ydzy_can_data._data_09[4];
	/*119*/_report_ydzy_data[pos++] = _ydzy_can_data._data_09[5];
	/*120*/_report_ydzy_data[pos++] = _ydzy_can_data._data_09[6];
	/*121*/_report_ydzy_data[pos++] = _ydzy_can_data._data_09[7];

	/*122*/_report_ydzy_data[pos++] = _ydzy_can_data._data_10[0];
	/*123*/_report_ydzy_data[pos++] = _ydzy_can_data._data_10[1];
	/*124*/_report_ydzy_data[pos++] = _ydzy_can_data._data_10[2];
	/*125*/_report_ydzy_data[pos++] = _ydzy_can_data._data_10[3];
	/*126*/_report_ydzy_data[pos++] = _ydzy_can_data._data_10[4];
	/*127*/_report_ydzy_data[pos++] = _ydzy_can_data._data_10[5];
	/*128*/_report_ydzy_data[pos++] = _ydzy_can_data._data_10[6];
	/*129*/_report_ydzy_data[pos++] = _ydzy_can_data._data_10[7];

	/*130*/_report_ydzy_data[pos++] = _ydzy_can_data._data_11[0];
	/*131*/_report_ydzy_data[pos++] = _ydzy_can_data._data_11[1];
	/*132*/_report_ydzy_data[pos++] = _ydzy_can_data._data_11[2];
	/*133*/_report_ydzy_data[pos++] = _ydzy_can_data._data_11[3];
	/*134*/_report_ydzy_data[pos++] = _ydzy_can_data._data_11[4];
	/*135*/_report_ydzy_data[pos++] = _ydzy_can_data._data_11[5];
	/*136*/_report_ydzy_data[pos++] = _ydzy_can_data._data_11[6];
	/*137*/_report_ydzy_data[pos++] = _ydzy_can_data._data_11[7];

	/*138*/_report_ydzy_data[pos++] = 0;
	/*139*/_report_ydzy_data[pos++] = 0;
	/*140*/_report_ydzy_data[pos++] = 0;
	/*141*/_report_ydzy_data[pos++] = 0;
	/*142*/_report_ydzy_data[pos++] = 0;
	/*143*/_report_ydzy_data[pos++] = 0;
	/*144*/_report_ydzy_data[pos++] = 0;
	/*145*/_report_ydzy_data[pos++] = 0;
}


void set_frame_data(void)
{
	if(g_car_config.protocol_type == HY_PROTOCOL)
		_set_hy_frame();
	else if(g_car_config.protocol_type == AX_PROTOCOL)
		_set_ax_frame();
	else if(g_car_config.protocol_type == XX_PROTOCOL)
		_set_xx_frame();
	else if(g_car_config.protocol_type == CGQC_PROTOCOL)
		_set_cgqc_frame();
	else if(g_car_config.protocol_type == DYKJ_PROTOCOL)
		_set_dykj_frame();
	else if(g_car_config.protocol_type == JSYT_PROTOCOL)
		_set_jsyt_frame();
	else if(g_car_config.protocol_type == VICT_PROTOCOL)
		_set_vict_frame();
	else if(g_car_config.protocol_type == YDZY_PROTOCOL)
		_set_ydzy_frame();
}

uint8_t *get_frame_data(void)
{
	if(g_car_config.protocol_type == HY_PROTOCOL)
		return _report_hy_data;
	else if(g_car_config.protocol_type == AX_PROTOCOL)
		return _report_ax_data;
	else if(g_car_config.protocol_type == XX_PROTOCOL)
		return _report_xx_data;
	else if(g_car_config.protocol_type == CGQC_PROTOCOL)
		return _report_cgqc_data;
	else if(g_car_config.protocol_type == DYKJ_PROTOCOL)
		return _report_dykj_data;
	else if(g_car_config.protocol_type == JSYT_PROTOCOL)
		return _report_jsyt_data;
	else if(g_car_config.protocol_type == VICT_PROTOCOL)
		return _report_vict_data;
	else if(g_car_config.protocol_type == YDZY_PROTOCOL)
		return _report_ydzy_data;
	
	return null;
}

uint16_t get_frame_size(void)
{
	if(g_car_config.protocol_type == HY_PROTOCOL)
		return sizeof(_report_hy_data);
	else if(g_car_config.protocol_type == AX_PROTOCOL)
		return sizeof(_report_ax_data);
	else if(g_car_config.protocol_type == XX_PROTOCOL)
		return sizeof(_report_xx_data);
	else if(g_car_config.protocol_type == CGQC_PROTOCOL)
		return sizeof(_report_cgqc_data);
	else if(g_car_config.protocol_type == DYKJ_PROTOCOL)
		return sizeof(_report_dykj_data);
	else if(g_car_config.protocol_type == JSYT_PROTOCOL)
		return sizeof(_report_jsyt_data);
	else if(g_car_config.protocol_type == VICT_PROTOCOL)
		return sizeof(_report_vict_data);
	else if(g_car_config.protocol_type == YDZY_PROTOCOL)
		return sizeof(_report_ydzy_data);

	return 0;
}


