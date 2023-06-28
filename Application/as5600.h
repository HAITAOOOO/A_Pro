#ifndef AS5600_H
#define	AS5600_H

#include "main.h"

#define as5600_addr 0x36
#define raw_angle_Hi 0x0C
#define raw_angle_Lo 0x0D

void as5600_init(void);
uint8_t as5600_read_one_byte_1(uint8_t addr);
uint8_t as5600_read_one_byte_2(uint8_t addr);
uint8_t as5600_read_one_byte_3(uint8_t addr);
uint8_t as5600_read_one_byte_4(uint8_t addr);
uint16_t as5600_read_raw_angle_1(void);
uint16_t as5600_read_raw_angle_2(void);
uint16_t as5600_read_raw_angle_3(void);
uint16_t as5600_read_raw_angle_4(void);

#endif
