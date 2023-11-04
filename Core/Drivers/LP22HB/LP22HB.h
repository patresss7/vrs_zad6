/*
 * LP22HB.h
 *
 *  Created on: Nov 4, 2023
 *      Author: patri
 */

#ifndef DRIVERS_LP22HB_LP22HB_H_
#define DRIVERS_LP22HB_LP22HB_H_
#include <stdint.h>

#define LPS25HB_I2C_ADDRESS_0 0xBA
#define LPS25HB_I2C_ADDRESS_1 0xB8

#define LPS25HB_REG_WHO_AM_I_ADDR 0x0F
#define LPS25HB_WHO_AM_I_VALUE 0b10110001

#define LPS25HB_REG_CTRL_REG1 0x10
#define LPS25HB_REG_PRESS_OUT_XL 0x28

void lps25hb_write_bytes(uint8_t register_address, uint8_t *data, uint8_t len, uint8_t i2c_address);
void lps25hb_read_bytes(uint8_t register_address, uint8_t *data, uint8_t len, uint8_t i2c_address);

uint8_t lps25hb_init(void);

float lps25hb_read_pressure(void);
float lps25hb_read_altitude(void);

#endif /* DRIVERS_LP22HB_LP22HB_H_ */

