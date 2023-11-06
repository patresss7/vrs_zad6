/*
 * hts221.c
 *
 *  Created on: Oct 27, 2023
 *      Author: laptop
 */

#include "hts221.h"
#include "i2c.h"
#include "usart.h"

static uint8_t address = HTS221_I2C_ADDRESS;
static float tx0, tx1, ty0, ty1; // temperature calibration data
static float hx0, hx1, hy0, hy1; // humidity calibration data

static float linear_interp(float x, float x0, float x1, float y0, float y1);



void hts221_write_byte(uint8_t reg_address, uint8_t *data,uint8_t len)
{
	write_multi_byte(data,1,address,reg_address);

}
uint8_t hts221_read_byte(uint8_t reg_address)
{
	uint8_t data = 0;
	read_multi_byte(&data,1,address,reg_address);
	return data;
}

void hts221_write_bytes(uint8_t reg_address, uint8_t *data,uint8_t len){
//	masterWriteMultiByte(data,len, slaveAddress, reg_address);
	write_multi_byte(data,len,address,reg_address);
}
void hts221_read_bytes(uint8_t reg_address, uint8_t *data,uint8_t len){
	if(len > 1)
		reg_address |= (1<<7);
//	masterReadMultiByte(data,len, slaveAddress, reg_address);
	read_multi_byte(data,len,address,reg_address);
}

uint8_t hts221_init()
{
	uint8_t id = hts221_read_byte(HTS221_REG_WHO_AM_I);
	if(id != HTS221_WHO_AM_I_VALUE)
	{
		return 0;
	}

	// read calibration data
	uint8_t h0_rh_x2 = hts221_read_byte(HTS221_REG_H0_rH_x2);
	uint8_t h1_rh_x2 = hts221_read_byte(HTS221_REG_H1_rH_x2);

	uint8_t t0_degC_x8_l = hts221_read_byte(HTS221_REG_T0_degC_x8);
	uint8_t t1_degC_x8_l = hts221_read_byte(HTS221_REG_T1_degC_x8);
	uint8_t t1_t0_msb = hts221_read_byte(HTS221_REG_T1_T0_MSB);

	uint8_t h0_t0_out_l = hts221_read_byte(HTS221_REG_H0_T0_OUT_L);
	uint8_t h0_t0_out_h = hts221_read_byte(HTS221_REG_H0_T0_OUT_H);
	uint8_t h1_t0_out_l = hts221_read_byte(HTS221_REG_H1_T0_OUT_L);
	uint8_t h1_t0_out_h = hts221_read_byte(HTS221_REG_H1_T0_OUT_H);

	uint8_t t0_out_l = hts221_read_byte(HTS221_REG_T0_OUT_L);
	uint8_t t0_out_h = hts221_read_byte(HTS221_REG_T0_OUT_H);
	uint8_t t1_out_l = hts221_read_byte(HTS221_REG_T1_OUT_L);
	uint8_t t1_out_h = hts221_read_byte(HTS221_REG_T1_OUT_H);

	int16_t t0_out = t0_out_l | (t0_out_h << 8);
	int16_t t1_out = t1_out_l | (t1_out_h << 8);

	int16_t t0_degC_x8 = t0_degC_x8_l | ((t1_t0_msb & 0b11) << 8);
	int16_t t1_degC_x8 = t1_degC_x8_l | ((t1_t0_msb & 0b1100) << (8 - 2));

	tx0 = (float)t0_out;
	tx1 = (float)t1_out;
	ty0 = (float)t0_degC_x8/8.0;
	ty1 = (float)t1_degC_x8/8.0;

	int16_t h0_t0_out = h0_t0_out_l | (h0_t0_out_h << 8);
	int16_t h1_t0_out = h1_t0_out_l | (h1_t0_out_h << 8);

	hx0 = (float)h0_t0_out;
	hx1 = (float)h1_t0_out;
	hy0 = (float)h0_rh_x2/2.0;
	hy1 = (float)h1_rh_x2/2.0;
	char message[120] = {0};
	int len = sprintf(message, "%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n", tx0, tx1, ty0, ty1, hx0, hx1, hy0, hy1);
	USART2_PutBuffer(message, len);
	LL_mDelay(1000);

	// set up sensor registers
	uint8_t ctrl_reg1 = 0b10000011;
	hts221_write_byte(HTS221_REG_CTRL_REG1, &ctrl_reg1,1);

	return 1;
}

uint8_t hts221_init_multi()
{
	uint8_t ID;
	hts221_read_bytes(HTS221_REG_WHO_AM_I, &ID, 1);
	if(ID != HTS221_WHO_AM_I_VALUE){
		return 0;
	}

	uint8_t t1_t0_msb;
	hts221_read_bytes(HTS221_REG_T1_T0_MSB, &t1_t0_msb, 1);

	uint8_t data[2];
	hts221_read_bytes(HTS221_REG_H0_rH_x2, &data, 1);
	hy0 = (float)data[0]/2.0;
	hts221_read_bytes(HTS221_REG_H1_rH_x2, &data, 1);
	hy1 = (float)data[0]/2.0;

	hts221_read_bytes(HTS221_REG_H0_T0_OUT_L, &data, 2);
	hx0 = (float)(data[0] | data[1] << 8);
	hts221_read_bytes(HTS221_REG_H1_T0_OUT_L, &data, 2);
	hx1 = (float)(data[0] | data[1] << 8);

	hts221_read_bytes(HTS221_REG_T0_degC_x8, &data, 1);
	ty0 = (float)(data[0] | ((t1_t0_msb & 0b11) << 8))/8.0;
	hts221_read_bytes(HTS221_REG_T1_degC_x8, &data, 1);
	ty1 = (float)(data[0] | ((t1_t0_msb & 0b1100) << (8 - 2)))/8.0;

	hts221_read_bytes(HTS221_REG_T0_OUT_L, &data, 2);
	tx0 = (float)(data[0] | data[1] << 8);
	hts221_read_bytes(HTS221_REG_T1_OUT_L, &data, 2);
	tx1 = (float)(data[0] | data[1] << 8);

	uint8_t ctrl_reg1 = 0b10000011;
	hts221_write_bytes(HTS221_REG_CTRL_REG1, &ctrl_reg1,1);
	return 1;
}

float hts221_get_temperature()
{
	// TODO calibration data
	uint8_t temp_l = hts221_read_byte(HTS221_REG_TEMP_OUT_L);
	uint8_t temp_h = hts221_read_byte(HTS221_REG_TEMP_OUT_H);

	int16_t temp = temp_l | (temp_h << 8);

	float temperature = linear_interp((float)temp, tx0, tx1, ty0, ty1);

	if(temperature > 120)
	{
		return 120.0;
	}
	if(temperature < -40)
	{
		return -40.0;
	}

	return temperature;
}

float hts221_get_humidity()
{
	// TODO calibration data
	uint8_t hum_l = hts221_read_byte(HTS221_REG_HUMIDITY_OUT_L);
	uint8_t hum_h = hts221_read_byte(HTS221_REG_HUMIDITY_OUT_H);

	int16_t hum = hum_l | (hum_h << 8);

	float humidity = linear_interp((float)hum, hx0, hx1, hy0, hy1);

	if(humidity > 100)
	{
		return 100.0;
	}
	if(humidity < 0)
	{
		return 0.0;
	}

	return humidity;
}

float hts221_get_temperature_multi()
{
	uint8_t data[2];
	uint16_t x;
	hts221_read_bytes(HTS221_REG_TEMP_OUT_L, &data, 2);
	x = data[0] | data[1]<<8;
	return linear_interp((float)x, tx0, tx1, ty0, ty1);
}

float hts221_get_humidity_multi()
{
	uint8_t data[2];
	uint16_t x;
	hts221_read_bytes(HTS221_REG_HUMIDITY_OUT_L, &data, 2);
	x = data[0] | data[1]<<8;
	return linear_interp((float)x, hx0, hx1, hy0, hy1);
}

static float linear_interp(float x, float x0, float x1, float y0, float y1)
{
	// https://www.vedantu.com/formula/linear-interpolation-formula
	return y0 + ((x-x0)*(y1-y0))/(x1-x0);
}
