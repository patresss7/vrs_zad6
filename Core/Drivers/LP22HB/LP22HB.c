/*
 * lp22hb.c
 *
 *  Created on: Nov 4, 2023
 *      Author: laptop
 */
#include "lp22hb.h"

static uint8_t slaveAddress = LPS22HB_I2C_ADDRESS_0;

void lp22hb_write_bytes(uint8_t *data, uint8_t reg_address, uint8_t len){
    write_multi_byte(data,len,slaveAddress,reg_address);
}
uint8_t lp22hb_read_byte(uint8_t reg_address)
{
	uint8_t data = 0;
	read_multi_byte(&data,1,slaveAddress,reg_address);
	return data;
}
void lp22hb_read_bytes(uint8_t register_address, uint8_t *data, uint8_t len, uint8_t i2c_address)
{
	read_multi_byte(data, len, i2c_address, register_address);
}

uint8_t lp22hb_init(){
    uint8_t ID = lp22hb_read_byte(LPS22HB_REG_WHO_AM_I_ADDR);
    if(ID != LPS22HB_WHO_AM_I_VALUE){
        slaveAddress = LPS22HB_I2C_ADDRESS_1;
        ID = lp22hb_read_byte(LPS22HB_REG_WHO_AM_I_ADDR);
        if(ID != LPS22HB_WHO_AM_I_VALUE)
            return 0;
    }
    uint8_t ctrl1 = 0b00110000;
    lp22hb_write_bytes(&ctrl1,LPS22HB_REG_CTRL_REG1,1);
    return 1;
}

float lp22hb_get_pressure(){
    uint8_t data[3];
    data[0] = lp22hb_read_byte(LPS22HB_REG_PRESS_OUT_XL);
    data[1] = lp22hb_read_byte(LPS22HB_REG_PRESS_OUT_XL + 1);
    data[2] = lp22hb_read_byte(LPS22HB_REG_PRESS_OUT_XL + 2);

    uint32_t pressure = data[2] << 16 | data[1] << 8 | data[0];

    return ((float)pressure)/4096;
}
float lp22hb_calculate_altitude(float pressure){
    return (float)44330 * (1 - pow(pressure/1013.25, 1/5.255));
}


