#ifndef __FLASH_H
#define __FALSH_H

#include "Dev.h"


#define FLASH_CONFIG_BASE_ADDR 0x800FC00

extern void read_data_from_flash(uint32_t flashAddr, uint8_t *pbuf, uint16_t len);
extern void write_data_to_rom(uint32_t flash_addr, uint8_t *Data, uint16_t len);
extern void work_program_from(uint32_t Addr);
extern void erases_flash_page(uint32_t Addr, uint8_t PageNum);
extern void init_flash_memory(void);

#endif
