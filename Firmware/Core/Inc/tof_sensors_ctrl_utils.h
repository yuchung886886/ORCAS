/*
 * tof_sensors_utils.h
 *
 *  Created on: Mar 7, 2025
 *      Author: yuchung886
 */

#ifndef INC_TOF_SENSORS_CTRL_UTILS_H_
#define INC_TOF_SENSORS_CTRL_UTILS_H_

uint8_t tof_sensors_init();

uint8_t tof_sensors_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf);

#define TOF_SENSORS_ROUTINES_MIN_PERIOD_US	500
uint8_t tof_sensors_perodic_routines();

uint8_t tof_sensors_isr(uint16_t GPIO_Pin);


#endif /* INC_TOF_SENSORS_CTRL_UTILS_H_ */
