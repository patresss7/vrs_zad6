/*
 * HTS221.c
 *
 *  Created on: Nov 4, 2023
 *      Author: patri
 */

#include "HTS221.h"
#include "i2c.h"

float tx0,tx1,ty0,ty1;
float hx0,hx1,hy0,hy1;

void hts221_write_bytes(uint8_t register_address, uint8_t *data, uint8_t len)
{
	write_multi_byte(data, len, HTS221_I2C_ADDRESS, register_address);

}
void hts221_read_bytes(uint8_t register_address, uint8_t *data, uint8_t len)
{
	read_multi_byte(data, len, HTS221_I2C_ADDRESS, register_address);
}

uint8_t hts221_init(void)
{
	uint8_t ID;
	hts221_read_bytes(HTS221_REG_WHO_AM_I, &ID, 1);
	if(ID != HTS221_WHO_AM_I_VALUE)
	{
		return 0;
	}

	uint8_t data[2];
	hts221_read_bytes(HTS221_REG_H0_rH_x2, &data, 1);
	hy0 = data[0]/2;
	hts221_read_bytes(HTS221_REG_H1_rH_x2, &data, 1);
	hy1 = data[0]/2;

	hts221_read_bytes(HTS221_REG_H0_T0_OUT_L, &data, 2);
	hx0 = data[0] | data[1] << 8;
	hts221_read_bytes(HTS221_REG_H1_T0_OUT_L, &data, 2);
	hx1 = data[0] | data[1] << 8;

	hts221_read_bytes(HTS221_REG_T0_degC_x8, &data, 1);
	ty0 = data[0]/8;
	hts221_read_bytes(HTS221_REG_T1_degC_x8, &data, 1);
	ty1 = data[0]/8;

	hts221_read_bytes(HTS221_REG_T0_OUT_L, &data, 2);
	tx0 = data[0] | data[1] << 8;
	hts221_read_bytes(HTS221_REG_T1_OUT_L, &data, 2);
	tx1 = data[0] | data[1] << 8;


	return 1;

}

float hts221_read_temp(void)
{
	float temp;
	uint8_t data[2];
	uint16_t x;
	hts221_read_bytes(HTS221_REG_TEMP_OUT_L, &data, 2);
	x = data[1] | data[0] << 8;
	return linear_interpolation(x, tx0, tx1, ty0, ty1);
}
float hts221_read_humid(void)
{
	float temp;
	uint8_t data[2];
	uint16_t x;
	hts221_read_bytes(HTS221_REG_HUMIDITY_OUT_L, &data, 2);
	x = data[1] | data[0] << 8;
	return linear_interpolation(x, hx0, hx1, hy0, hy1);
}

float linear_interpolation(uint16_t x,float xo,float x1,float y0,float y1)
{
	if (x < xo || x > x1) {
	        // Handle the out-of-range case as needed (e.g., return an error code).
	        // You may choose to return a specific value or handle it differently.
	        // For simplicity, let's return 0 in this example.
	        return 0.0;
	    }

	    // Perform linear interpolation
	    // Calculate the interpolation factor
	    float factor = ((float)x - xo) / (x1 - xo);

	    // Calculate the interpolated value
	    float interpolatedValue = y0 + factor * (y1 - y0);

	    return interpolatedValue;
}
