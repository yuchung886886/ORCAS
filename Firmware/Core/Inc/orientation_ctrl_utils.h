/*
 * orientation_ctrl_utils.h
 *
 *  Created on: Feb 3, 2025
 *      Author: yuchung886
 */

#ifndef INC_ORIENTATION_CTRL_UTILS_H_
#define INC_ORIENTATION_CTRL_UTILS_H_

uint8_t orientation_ctrl_init();

uint8_t orientation_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf);

#define ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US	250
uint8_t orientation_ctrl_perodic_routines();

uint8_t orientation_ctrl_isr(uint16_t GPIO_Pin);

extern uint16_t curr_pan_angle_in_steps;
extern float pan_angle_per_step;

#endif /* INC_ORIENTATION_CTRL_UTILS_H_ */
