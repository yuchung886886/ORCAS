/*
 * fire_ctrl_utils.c
 *
 *  Created on: Jan 21, 2025
 *      Author: yuchung886
 */
#include "main.h"
#include "fire_ctrl_utils.h"
#include "uart_cmds_handle.h"
#include "mma845x.h"
#include "error_code.h"

#define FIRE_CTRL_ROUTINES_GRP1_PERIOD_US 10000
#define FIRE_CTRL_ROUTINES_GRP2_PERIOD_US FIRE_CTRL_ROUTINES_MIN_PERIOD_US
uint8_t fire_ctrl_perodic_routines_timer = 0;

uint8_t fire_ctrl_status = STATUS__SAFETY_EN;

#define AEG_MOTOR_STATE__READY				0
#define AEG_MOTOR_STATE__PULLING_PISTON		1
#define AEG_MOTOR_STATE__RELEASING_PISTON	2
uint8_t aeg_motor_state = AEG_MOTOR_STATE__READY;

#define	AEG_PISTON_PULLING_TIMEOUT_MS		500
// #define AEG_PISTON_RELEASING_TIMEOUT_MS		100
#define AEG_MOTOR_ON_DEBOUNCE_TIMEOUT_MS	20
uint16_t aeg_piston_released_timeout_ms = 100;
uint16_t aeg_motor_timer_ms = 0;

uint8_t aeg_motor_duty__auto = 100;
uint8_t aeg_motor_duty__semi = 60;
uint8_t fire_count = 0;

uint8_t aeg_tracer_duty = 0;
#define AEG_TRACER_DIMMING_DELAY_MS	50
uint16_t aeg_tracer_timer_ms = 0;

#define RED_DOT_FLASH_HALF_PERIOD_MS	50
uint16_t red_dot_timer_ms = 0;

#define AMMO_FEERER_MOTOR_DIR_CW	0	// Load ammo
#define AMMO_FEERER_MOTOR_DIR_CCW	1	// Un-load ammo
#define AMMO_FEERER_MOTOR_STEPS_PER_SHOT_DEFAULT	220
uint16_t ammo_feeder_motor_steps_per_shot = AMMO_FEERER_MOTOR_STEPS_PER_SHOT_DEFAULT;
uint16_t ammo_feeder_motor_steps_remained = 0;

uint8_t targeting_led_duty = 0;

extern TIM_HandleTypeDef htim3;
#define AEG_MOTOR_PWM TIM_CHANNEL_1
#define AEG_TRACER_PWM TIM_CHANNEL_2
#define SEARCHLIGHT_PWM TIM_CHANNEL_4
TIM_OC_InitTypeDef sConfigOC = {TIM_OCMODE_PWM1, 0, TIM_OCPOLARITY_HIGH, 0, TIM_OCFAST_DISABLE, 0, 0};

static uint8_t aeg_motor_ctrl();
static uint8_t aeg_tracer_ctrl();
static uint8_t aeg_safety_ctrl();
static uint8_t ammo_feeder_ctrl();

uint8_t fire_ctrl_init(){
	sConfigOC.Pulse = 0;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
	HAL_TIM_PWM_Start(&htim3, AEG_MOTOR_PWM);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_TRACER_PWM);
	HAL_TIM_PWM_Start(&htim3, AEG_TRACER_PWM);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, SEARCHLIGHT_PWM);
	HAL_TIM_PWM_Start(&htim3, SEARCHLIGHT_PWM);
	return 0;
}

uint8_t fire_ctrl_cmds_dispatch(uint8_t* cmd_buf, uint8_t* ack_buf){
	switch(cmd_buf[CMD_CODE_INDEX]){
		case CMD_CODE__GET_FIRE_CTRL_STATUS:
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			ack_buf[ACK_PARAMS_INDEX] = fire_ctrl_status;
			break;
		case CMD_CODE__SET_FIRE_COUNT:
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			if(fire_ctrl_status & STATUS__PULLING_PISTON_TIMEOUT){
				fire_ctrl_status &= ~STATUS__PULLING_PISTON_TIMEOUT;
				ack_buf[ACK_CODE_INDEX] = ERR__AEG_PISTON_PULLING_TIMEOUT;
			}
			fire_count = cmd_buf[CMD_PARAMS_INDEX];

			break;
		case CMD_CODE__SET_FIRE_CTRL_CONFIG:
			if((cmd_buf[CMD_PARAMS_INDEX] <= 100) &&
			   (cmd_buf[CMD_PARAMS_INDEX] > 0)){
				aeg_motor_duty__semi = cmd_buf[CMD_PARAMS_INDEX];
			}
			if((cmd_buf[CMD_PARAMS_INDEX + 1] <= 100) &&
			   (cmd_buf[CMD_PARAMS_INDEX + 1] > 0)){
				aeg_motor_duty__auto = cmd_buf[CMD_PARAMS_INDEX + 1];
			}
			aeg_piston_released_timeout_ms = cmd_buf[CMD_PARAMS_INDEX + 2] * 10;
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			break;
		case CMD_CODE__SET_FIRE_SAFETY:
			if(cmd_buf[CMD_PARAMS_INDEX] == 1){
				fire_count = 0;
				fire_ctrl_status |= STATUS__SAFETY_EN;
			}else{
				fire_ctrl_status &= ~STATUS__SAFETY_EN;
			}
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			break;
		case CMD_CODE__SET_AMMO_FEEDER_CONFIG:
			ammo_feeder_motor_steps_per_shot = cmd_buf[CMD_PARAMS_INDEX] * 10;
			break;
		case CMD_CODE__SET_AMMO_FEEDING:
			if(cmd_buf[CMD_PARAMS_INDEX] == AMMO_FEERER_MOTOR_DIR_CCW){
				// Un-Load ammo
				HAL_GPIO_WritePin(FEEDER_MOTOR_DIR_GPIO_Port, FEEDER_MOTOR_DIR_Pin, GPIO_PIN_SET);
				fire_ctrl_status |= STATUS__FEEDER_MOTOR_DIR;
			}else{
				// Load ammo
				HAL_GPIO_WritePin(FEEDER_MOTOR_DIR_GPIO_Port, FEEDER_MOTOR_DIR_Pin, GPIO_PIN_RESET);
				fire_ctrl_status &= ~STATUS__FEEDER_MOTOR_DIR;
			}
			ammo_feeder_motor_steps_remained = (cmd_buf[CMD_PARAMS_INDEX + 1] << 8) | cmd_buf[CMD_PARAMS_INDEX + 2];
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			break;
		case CMD_CODE__SET_SEARCHLIGHT_PWM:
			if((cmd_buf[CMD_PARAMS_INDEX] <= 100) &&
			   (cmd_buf[CMD_PARAMS_INDEX] >= 0)){
				targeting_led_duty = cmd_buf[CMD_PARAMS_INDEX];
				if(targeting_led_duty == 0){
					fire_ctrl_status &= ~STATUS__TARGETING_LED_EN;
				}else{
					fire_ctrl_status |= STATUS__TARGETING_LED_EN;
				}
				sConfigOC.Pulse = targeting_led_duty;
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, SEARCHLIGHT_PWM);
			}
			ack_buf[ACK_CODE_INDEX] = ACK_CODE__SUCCESS;
			break;
		default:
			break;
	}
	return 0;
}

uint8_t fire_ctrl_perodic_routines(){
	fire_ctrl_perodic_routines_timer++;
	if(fire_ctrl_perodic_routines_timer == (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / FIRE_CTRL_ROUTINES_MIN_PERIOD_US)){
		fire_ctrl_perodic_routines_timer = 0;
	}

	if(fire_ctrl_perodic_routines_timer % (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / FIRE_CTRL_ROUTINES_MIN_PERIOD_US) == 0){
		aeg_motor_ctrl();
		aeg_tracer_ctrl();
		aeg_safety_ctrl();
	}
	if(fire_ctrl_perodic_routines_timer % (FIRE_CTRL_ROUTINES_GRP2_PERIOD_US / FIRE_CTRL_ROUTINES_MIN_PERIOD_US) == 0){
		ammo_feeder_ctrl();
	}

	return 0;
}

uint8_t fire_ctrl_isr(uint16_t GPIO_Pin){
	if(GPIO_Pin == AEG_PISTON_ENDSTOP_Pin){
		sConfigOC.Pulse = 0;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
		fire_ctrl_status &= ~(STATUS__AEG_MOTOR_EN);

		if((aeg_motor_state == AEG_MOTOR_STATE__PULLING_PISTON) &&
		   (aeg_motor_timer_ms >= AEG_MOTOR_ON_DEBOUNCE_TIMEOUT_MS)){
			aeg_motor_timer_ms = 0;
			aeg_motor_state = AEG_MOTOR_STATE__RELEASING_PISTON;

			aeg_tracer_timer_ms = 0;
			aeg_tracer_duty = 100;
			sConfigOC.Pulse = aeg_tracer_duty;
			HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_TRACER_PWM);

			// Load ammo
			HAL_GPIO_WritePin(FEEDER_MOTOR_DIR_GPIO_Port, FEEDER_MOTOR_DIR_Pin, GPIO_PIN_RESET);
			fire_ctrl_status &= ~STATUS__FEEDER_MOTOR_DIR;
			ammo_feeder_motor_steps_remained += ammo_feeder_motor_steps_per_shot;
		}
	}

	return 0;
}

static uint8_t aeg_motor_ctrl(){
	if(fire_count){
		if(!(fire_ctrl_status & STATUS__SAFETY_EN)){
			switch(aeg_motor_state){
			case AEG_MOTOR_STATE__READY:
				if(fire_count == 1){
					sConfigOC.Pulse = aeg_motor_duty__semi;
				}else{
					sConfigOC.Pulse = aeg_motor_duty__auto;
				}
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
				fire_ctrl_status |= STATUS__AEG_MOTOR_EN;
				aeg_motor_timer_ms = 0;
				aeg_motor_state = AEG_MOTOR_STATE__PULLING_PISTON;
				break;
			case AEG_MOTOR_STATE__PULLING_PISTON:
				aeg_motor_timer_ms += (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / 1000);
				if(aeg_motor_timer_ms >= AEG_PISTON_PULLING_TIMEOUT_MS){
					sConfigOC.Pulse = 0;
					HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
					fire_ctrl_status &= ~(STATUS__AEG_MOTOR_EN);
					fire_count = 0;
					aeg_motor_timer_ms = 0;
					aeg_motor_state = AEG_MOTOR_STATE__READY;
					fire_ctrl_status |= STATUS__PULLING_PISTON_TIMEOUT;
				}
				break;
			case AEG_MOTOR_STATE__RELEASING_PISTON:
				aeg_motor_timer_ms += (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / 1000);
				if(aeg_motor_timer_ms >= aeg_piston_released_timeout_ms){
					if(--fire_count){
						if(fire_count == 1){
							sConfigOC.Pulse = aeg_motor_duty__semi;
						}else{
							sConfigOC.Pulse = aeg_motor_duty__auto;
						}
						HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
						fire_ctrl_status |= STATUS__AEG_MOTOR_EN;
						aeg_motor_timer_ms = 0;
						aeg_motor_state = AEG_MOTOR_STATE__PULLING_PISTON;
					}else{
						sConfigOC.Pulse = 0;
						HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_MOTOR_PWM);
						fire_ctrl_status &= ~(STATUS__AEG_MOTOR_EN);
						aeg_motor_timer_ms = 0;
						aeg_motor_state = AEG_MOTOR_STATE__READY;
					}
				}
				break;
			default:
				break;
			}
		}
	}
	return 0;
}

static uint8_t aeg_tracer_ctrl(){
	if(aeg_tracer_duty){
		if(aeg_tracer_timer_ms > AEG_TRACER_DIMMING_DELAY_MS){
			if(fire_count > 1){
				aeg_tracer_duty = 0;
				sConfigOC.Pulse = aeg_tracer_duty;
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_TRACER_PWM);
			}else{
				aeg_tracer_duty = aeg_tracer_duty - 1;
				// aeg_tracer_duty = 0;
				sConfigOC.Pulse = aeg_tracer_duty;
				HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, AEG_TRACER_PWM);
			}
		}
		aeg_tracer_timer_ms += (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / 1000);
	}
	return 0;
}

static uint8_t aeg_safety_ctrl(){
	if(fire_ctrl_status & STATUS__SAFETY_EN){
		if(fire_count){
			red_dot_timer_ms += (FIRE_CTRL_ROUTINES_GRP1_PERIOD_US / 1000);
			if(red_dot_timer_ms % RED_DOT_FLASH_HALF_PERIOD_MS == 0){
				// HAL_GPIO_TogglePin(RED_DOT_EN_GPIO_Port, RED_DOT_EN_Pin);
				if(fire_ctrl_status & STATUS__RED_DOT_EN){
					fire_ctrl_status &= ~STATUS__RED_DOT_EN;
					HAL_GPIO_WritePin(RED_DOT_EN_GPIO_Port, RED_DOT_EN_Pin, GPIO_PIN_RESET);
				}else{
					fire_ctrl_status |= STATUS__RED_DOT_EN;
					HAL_GPIO_WritePin(RED_DOT_EN_GPIO_Port, RED_DOT_EN_Pin, GPIO_PIN_SET);
				}
				if(red_dot_timer_ms >= RED_DOT_FLASH_HALF_PERIOD_MS * 6){
					red_dot_timer_ms = 0;
					fire_count--;
				}
			}
		}else{
			if(fire_ctrl_status & STATUS__RED_DOT_EN){
				fire_ctrl_status &= ~STATUS__RED_DOT_EN;
				HAL_GPIO_WritePin(RED_DOT_EN_GPIO_Port, RED_DOT_EN_Pin, GPIO_PIN_RESET);
			}
		}
	}else{
		if(!(fire_ctrl_status & STATUS__RED_DOT_EN)){
			fire_ctrl_status |= STATUS__RED_DOT_EN;
			HAL_GPIO_WritePin(RED_DOT_EN_GPIO_Port, RED_DOT_EN_Pin, GPIO_PIN_SET);
		}
	}
	return 0;
}

#define STATE_SLEEP		0
#define STATE_ACTIVE	1
uint8_t ammo_feeder_state = STATE_SLEEP;
#define AMMO_FEEDER_WAKEUP_DELAY_US	1000
uint8_t ammo_feeder_timer = 0;
static uint8_t ammo_feeder_ctrl(){
	switch(ammo_feeder_state){
	case STATE_SLEEP:
		if(ammo_feeder_motor_steps_remained){
			HAL_GPIO_WritePin(FEEDER_MOTOR_SLEEP_GPIO_Port, FEEDER_MOTOR_SLEEP_Pin, GPIO_PIN_SET);
			ammo_feeder_timer++;
			if(ammo_feeder_timer > (AMMO_FEEDER_WAKEUP_DELAY_US / FIRE_CTRL_ROUTINES_GRP2_PERIOD_US)){
				ammo_feeder_timer = 0;
				ammo_feeder_state = STATE_ACTIVE;
			}
		}
		break;
	case STATE_ACTIVE:
		if(ammo_feeder_motor_steps_remained){
			if(fire_ctrl_status & STATUS__FEEDER_MOTOR_STEP){
				HAL_GPIO_WritePin(FEEDER_MOTOR_STEP_GPIO_Port, FEEDER_MOTOR_STEP_Pin, GPIO_PIN_RESET);
				fire_ctrl_status &= ~STATUS__FEEDER_MOTOR_STEP;
				ammo_feeder_motor_steps_remained--;
				if(!ammo_feeder_motor_steps_remained){
					HAL_GPIO_WritePin(FEEDER_MOTOR_SLEEP_GPIO_Port, FEEDER_MOTOR_SLEEP_Pin, GPIO_PIN_RESET);
					ammo_feeder_state = STATE_SLEEP;
				}
			}else{
				HAL_GPIO_WritePin(FEEDER_MOTOR_STEP_GPIO_Port, FEEDER_MOTOR_STEP_Pin, GPIO_PIN_SET);
				fire_ctrl_status |= STATUS__FEEDER_MOTOR_STEP;
			}
		}
		break;
	default:
		break;
	}
	return 0;
}


