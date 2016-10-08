#ifndef __TOOL_H
#define __TOOL_H

#include "Dev.h"

extern char* tool_get_str_gnrmc(char* full_str, char *out_str);
extern uint8_t tool_get_gps_status(char *status);
extern uint8_t tool_get_str_gnrmc_keyvalue(char *str_gnrmc, char *ddmmyy, char *hhmmss, char *status, char *longitude, char *latitude, char *speed, char *dir);
extern uint32_t tool_convert_gps_longitude_latitude(char *input);
extern uint16_t tool_convert_gps_speed(char *input);
extern uint16_t tool_convert_gps_ground_course(char *input);
extern uint32_t tool_convert_gps_ddmmyy(char *input);
extern uint32_t tool_convert_gps_hhmmss(char *input);

extern uint16_t cal_crc16(uint8_t *buf, uint16_t len);

extern char *tool_get_key_str(char *full_str, char* key_str, int *plen);
extern u32 tool_get_key_int(char* full_str, char* key_str);


#endif
