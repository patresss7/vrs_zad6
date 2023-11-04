/*
 * HTS221.h
 *
 *  Created on: Nov 4, 2023
 *      Author: patri
 */



#ifndef DRIVERS_HTS221_HTS221_H_
#define DRIVERS_HTS221_HTS221_H_

#include <stdio.h>
#include <stdint.h>


#define HTS221_I2C_ADDRESS 0xBE

#define HTS221_WHO_AM_I_VALUE 0b10111100
#define HTS221_REG_WHO_AM_I 0x0F
#define HTS221_REG_CTRL_REG1 0x20

#define HTS221_REG_HUMIDITY_OUT_L 0x28
#define HTS221_REG_TEMP_OUT_L 0x2A

// calibration registers
#define HTS221_REG_H0_rH_x2 0x30
#define HTS221_REG_H1_rH_x2 0x31
#define HTS221_REG_T0_degC_x8 0x32
#define HTS221_REG_T1_degC_x8 0x33
#define HTS221_REG_T1_T0_MSB 0x35
#define HTS221_REG_H0_T0_OUT_L 0x36
#define HTS221_REG_H1_T0_OUT_L 0x3A
#define HTS221_REG_T0_OUT_L 0x3C
#define HTS221_REG_T1_OUT_L 0x3E

void hts221_write_bytes(uint8_t register_address, uint8_t *data, uint8_t len);
void hts221_read_bytes(uint8_t register_address, uint8_t *data, uint8_t len);

uint8_t hts221_init(void);

float hts221_read_temp(void);
float hts221_read_humid(void);

float linear_interpolation(uint16_t x,float xo,float x1,float y0,float y1);


#endif /* DRIVERS_HTS221_HTS221_H_ */
