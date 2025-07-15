/*
 * mma845x.h
 *
 *  Created on: Feb 3, 2025
 *      Author: yuchung886
 */

#ifndef INC_MMA845X_H_
#define INC_MMA845X_H_

uint8_t mma845x_init(I2C_HandleTypeDef *hi2c);
uint8_t mma845x_get_xyz(I2C_HandleTypeDef *hi2c, int16_t* x, int16_t* y, int16_t* z);

extern uint8_t mma845x_ret;

#endif /* INC_MMA845X_H_ */
