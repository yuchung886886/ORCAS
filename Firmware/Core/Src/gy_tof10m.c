/*
 * gy_tof10m.c
 *
 *  Created on: Mar 12, 2025
 *      Author: yuchung886
 */

#include "main.h"
#include "error_code.h"

#define I2C_TIMEOUT			10

#define GYTOF10M_SLAVE_ADDR	0xA4

#define REG_ADDR__REFRESH_RATE	0x02
#define REFRESH_RATE__1HZ			0x00
#define REFRESH_RATE__10HZ			0x01
#define REFRESH_RATE__50HZ			0x02
#define REFRESH_RATE__100HZ			0x03
#define REFRESH_RATE__200HZ			0x04

#define REG_ADDR__OUTPUT_MODE	0x03
#define OUTPUT_MODE__CONTINUOUS		0x00
#define OUTPUT_MODE__INTERRUPT		0x01
#define OUTPUT_MODE__POLLING		0x02

#define REG_ADDR__DISTANCE_H	0x08
#define REG_ADDR__DISTANCE_L	0x09

uint8_t gy_tof10m_ctrl_buf[10] = {0};

uint8_t gy_tof10m__init(I2C_HandleTypeDef *hi2c){
	uint8_t ret = 0;

	gy_tof10m_ctrl_buf[0] = REG_ADDR__REFRESH_RATE;
	gy_tof10m_ctrl_buf[1] = REFRESH_RATE__10HZ;
	if(!ret){ret = HAL_I2C_Master_Transmit(hi2c, GYTOF10M_SLAVE_ADDR, gy_tof10m_ctrl_buf, 2, I2C_TIMEOUT);}
	gy_tof10m_ctrl_buf[0] = REG_ADDR__OUTPUT_MODE;
	gy_tof10m_ctrl_buf[1] = OUTPUT_MODE__CONTINUOUS;
	if(!ret){ret = HAL_I2C_Master_Transmit(hi2c, GYTOF10M_SLAVE_ADDR, gy_tof10m_ctrl_buf, 2, I2C_TIMEOUT);}
	if(ret){
		return ERR__GY_TOF10M_COMM_FAIL;
	}else{
		return 0;
	}
}

uint8_t gy_tof10m__get_range(I2C_HandleTypeDef *hi2c, uint16_t* range){
	uint8_t ret = 0;

	gy_tof10m_ctrl_buf[0] = REG_ADDR__DISTANCE_H;

	// if(!ret){ret = HAL_I2C_Master_Transmit(hi2c, GYTOF10M_SLAVE_ADDR, gy_tof10m_ctrl_buf, 1, I2C_TIMEOUT);}
	if(!ret){ret = HAL_I2C_Master_Receive(hi2c, GYTOF10M_SLAVE_ADDR, gy_tof10m_ctrl_buf, 10, I2C_TIMEOUT);}
	/*
	HAL_I2C_Master_Seq_Transmit_IT(hi2c, GYTOF10M_SLAVE_ADDR, gy_tof10m_ctrl_buf, 1, I2C_FIRST_FRAME);
	while(hi2c->State != HAL_I2C_STATE_READY){}
	HAL_I2C_Maste r_Seq_Receive_IT(hi2c, GYTOF10M_SLAVE_ADDR, gy_tof10m_ctrl_buf, 2, I2C_LAST_FRAME);
	while(hi2c->State != HAL_I2C_STATE_READY){}
	*/
	if(ret){
		return ERR__GY_TOF10M_COMM_FAIL;
	}else{
		*range = (gy_tof10m_ctrl_buf[REG_ADDR__DISTANCE_H] << 8) | gy_tof10m_ctrl_buf[REG_ADDR__DISTANCE_L];
		return 0;
	}
}



