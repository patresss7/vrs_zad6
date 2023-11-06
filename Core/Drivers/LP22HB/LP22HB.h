/*
 * lp22hb.h
 *
 *  Created on: Nov 4, 2023
 *      Author: laptop
 */

#ifndef DRIVERS_LP22HB_LP22HB_H_
#define DRIVERS_LP22HB_LP22HB_H_

#include "stdint.h"
#include "i2c.h"
#include <math.h>

#define LPS22HB_I2C_ADDRESS_0       0xBA
#define LPS22HB_I2C_ADDRESS_1       0xB8

#define LPS22HB_REG_WHO_AM_I_ADDR   0x0F
#define LPS22HB_WHO_AM_I_VALUE      0b10110001

#define LPS22HB_REG_CTRL_REG1       0x10
#define LPS22HB_REG_PRESS_OUT_XL    0x28


void lp22hb_write_bytes(uint8_t *data, uint8_t reg_address, uint8_t len);
void lp22hb_read_bytes(uint8_t register_address, uint8_t *data, uint8_t len, uint8_t i2c_address);

uint8_t lp22hb_init();
uint8_t lp22hb_init_multi();

float lp22hb_get_pressure();
float lp22hb_get_pressure_multi();
float lp22hb_calculate_altitude(float pressure);


#endif /* DRIVERS_LP22HB_LP22HB_H_ */
