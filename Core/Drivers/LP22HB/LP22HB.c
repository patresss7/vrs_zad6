/*
 * LP22HB.c
 *
 *  Created on: Nov 4, 2023
 *      Author: patri
 */

#include "LP22HB.h"
#include "i2c.h"
#include <math.h>


uint8_t actual_address = LPS25HB_I2C_ADDRESS_0;

void lps25hb_write_bytes(uint8_t register_address, uint8_t *data, uint8_t len, uint8_t i2c_address)
{
	write_multi_byte(data, len, i2c_address, register_address);
}
void lps25hb_read_bytes(uint8_t register_address, uint8_t *data, uint8_t len, uint8_t i2c_address)
{
	read_multi_byte(data, len, i2c_address, register_address);
}

uint8_t lps25hb_init(void)
{
	uint8_t ID;
	lps25hb_read_bytes(LPS25HB_REG_WHO_AM_I_ADDR, &ID, 1, actual_address);
	if(ID != LPS25HB_WHO_AM_I_VALUE)
	{
		actual_address = LPS25HB_I2C_ADDRESS_1;
		lps25hb_read_bytes(LPS25HB_REG_WHO_AM_I_ADDR, &ID, 1, actual_address);
		if(ID != LPS25HB_WHO_AM_I_VALUE)
		{
			return 0;
		}
	}
	lps25hb_write_bytes(LPS25HB_REG_CTRL_REG1, 0b00110000, 1, actual_address);
	return 1;
}

float lps25hb_read_pressure(void)
{
	float result = 0;
	uint8_t data[3];
	lps25hb_read_bytes(LPS25HB_REG_PRESS_OUT_XL, &data, 3, actual_address);
	int32_t pressure_raw = (int32_t)((data[2] << 16) | (data[1] << 8) | data[0]);
	result = pressure_raw / 4096;
	return result;

}

float lps25hb_read_altitude(void)
{

	const float SEA_LEVEL_PRESSURE_HPA = 1013.25; // Standard sea-level pressure in hPa

	float altitude = 0;
	float pressure = lps25hb_read_pressure();

	 altitude = 44330.0 * (1.0 - pow((pressure / SEA_LEVEL_PRESSURE_HPA), 0.1903));

	 return altitude;

}
