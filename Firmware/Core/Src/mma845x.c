/*
 * mma845x.c
 *
 *  Created on: Feb 3, 2025
 *      Author: yuchung886
 */

#include "main.h"
#include "error_code.h"

#define I2C_TIMEOUT			10

#define MMA845X_I2C_ADDR	0x38

#define REG_ADDR__STATUS			0x00
#define REG_ADDR__OUT_X_MSB			0x01
#define REG_ADDR__OUT_X_LSB			0x02
#define REG_ADDR__OUT_Y_MSB			0x03
#define REG_ADDR__OUT_Y_LSB			0x04
#define REG_ADDR__OUT_Z_MSB			0x05
#define REG_ADDR__OUT_Z_LSB			0x06

#define REG_ADDR__XYZ_DATA_CONFIG	0x0E
#define XYZ_DATA_CONFIG__FS_2G		0x00
#define XYZ_DATA_CONFIG__FS_4G		0x01
#define XYZ_DATA_CONFIG__FS_8G		0x02

#define REG_ADDR__HP_FILTER_CUTOFF	0x0F
#define HP_FILTER_CUTOFF__SEL0		0x00

#define REG_ADDR__CTRL_REG1			0x2A
#define CTRL_REG1__DR_50HZ			0x20
#define CTRL_REG1__DR_6P25HZ		0x30
#define CTRL_REG1__ACTIVE			0x01

#define REG_ADDR__CTRL_REG2			0x2B
#define CTRL_REG2__MODS_NORMAL		0x00

uint8_t mma845x_ctrl_buf[8] = {0};

uint8_t mma845x_init(I2C_HandleTypeDef *hi2c){
	uint8_t ret = 0;

	mma845x_ctrl_buf[0] = REG_ADDR__CTRL_REG1;
	mma845x_ctrl_buf[1] = 0; // Set MMA845X to standby mode.
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);

	mma845x_ctrl_buf[0] = REG_ADDR__XYZ_DATA_CONFIG;
	mma845x_ctrl_buf[1] = XYZ_DATA_CONFIG__FS_2G;
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);

	mma845x_ctrl_buf[0] = REG_ADDR__CTRL_REG2;
	mma845x_ctrl_buf[1] = CTRL_REG2__MODS_NORMAL;
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);

	/*
	mma845x_ctrl_buf[0] = REG_ADDR__HP_FILTER_CUTOFF;
	mma845x_ctrl_buf[1] = HP_FILTER_CUTOFF__SEL0;
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);
	*/

	mma845x_ctrl_buf[0] = REG_ADDR__CTRL_REG1;
	mma845x_ctrl_buf[1] = CTRL_REG1__DR_6P25HZ | CTRL_REG1__ACTIVE;
	if(!ret) ret = HAL_I2C_Master_Transmit(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, 2, I2C_TIMEOUT);

	if(ret){
		return ERR__MMA845X_COMM_FAIL;
	}else{
		return 0;
	}
}

uint8_t mma845x_ret = 0;
uint8_t mma845x_get_xyz(I2C_HandleTypeDef *hi2c, int16_t* x, int16_t* y, int16_t* z){
	uint8_t ret = 0;

	if(!ret) ret = HAL_I2C_Master_Receive(hi2c, MMA845X_I2C_ADDR, mma845x_ctrl_buf, REG_ADDR__OUT_Z_LSB - REG_ADDR__STATUS + 1, I2C_TIMEOUT);
	if(!ret){
		*x = (int16_t)((mma845x_ctrl_buf[1] << 8) | mma845x_ctrl_buf[2]) / 4;
		*y = (int16_t)((mma845x_ctrl_buf[3] << 8) | mma845x_ctrl_buf[4]) / 4;
		*z = (int16_t)((mma845x_ctrl_buf[5] << 8) | mma845x_ctrl_buf[6]) / 4;
		return 0;
	}else{
		mma845x_ret = ret;
		return ERR__MMA845X_COMM_FAIL;
	}
}
