/*
 * gy_tof10m.h
 *
 *  Created on: Mar 12, 2025
 *      Author: yuchung886
 */

#ifndef INC_GY_TOF10M_H_
#define INC_GY_TOF10M_H_

uint8_t gy_tof10m__init(I2C_HandleTypeDef *hi2c);
uint8_t gy_tof10m__get_range(I2C_HandleTypeDef *hi2c, uint16_t* range);

#endif /* INC_GY_TOF10M_H_ */
