/*
 * fire_ctrl_utils.h
 *
 *  Created on: Jan 21, 2025
 *      Author: yuchung886
 */

#ifndef INC_FIRE_CTRL_UTILS_H_
#define INC_FIRE_CTRL_UTILS_H_

uint8_t fire_ctrl_init();

uint8_t fire_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf);

#define FIRE_CTRL_ROUTINES_MIN_PERIOD_US	1000
uint8_t fire_ctrl_perodic_routines();

uint8_t fire_ctrl_isr(uint16_t GPIO_Pin);

#define STATUS__SAFETY_EN					0x01
#define STATUS__AEG_MOTOR_EN				0x02
#define STATUS__PULLING_PISTON_TIMEOUT		0x04
#define STATUS__RED_DOT_EN					0x08
#define STATUS__FEEDER_MOTOR_DIR			0x10
#define STATUS__FEEDER_MOTOR_STEP			0x20
#define STATUS__TARGETING_LED_PWM_UPDATED	0x40
#define STATUS__TARGETING_LED_EN			0x80
extern uint8_t fire_ctrl_status;

#endif /* INC_FIRE_CTRL_UTILS_H_ */
