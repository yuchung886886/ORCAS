/*
 * srf02.h
 *
 *  Created on: Mar 7, 2025
 *      Author: yuchung886
 */

#ifndef INC_GY_US42V2_H_
#define INC_GY_US42V2_H_

uint8_t gy_us42v2__start_ranging(I2C_HandleTypeDef *hi2c);
uint8_t gy_us42v2__get_range(I2C_HandleTypeDef *hi2c, uint16_t* range);

#endif /* INC_GY_US42V2_H_ */
