#include "motor.h"
#include <string.h>

void motor_init_pins(motor_t *m, motor_cfg_t *cfg){
	m->cfg = *cfg;
}
void motor_init_timer(motor_t *m, TIM_HandleTypeDef* htim,uint32_t channel, uint8_t pwmMaxPrecent){
	trapezoidal_func_init(&m->trap,htim,channel,pwmMaxPrecent);
}
//set activity
void motor_activity(motor_t *m, motor_status_t status){
	m->Status = status;
	m->flag.status_changed = 1;
}
void motor_set_dir(motor_t *m, motor_dir_t dir){
	m->dir = dir;
	m->flag.dir_changed = 1;
}
void motor_start(motor_t *m, motor_start_mode_t mode){
	m->startMode = mode;
	m->flag.start_pending = 1;
}
void motor_stop(motor_t *m, motor_stop_mode_t mode){
	m->stopMode = mode;
	m->flag.stop_pending = 1;
}
//do changes

void motor_update(motor_t *m, uint32_t now_ms){
	if(m->flag.dir_changed){
		m->flag.dir_changed = RESET;
		HAL_GPIO_TogglePin(m->cfg.pin_dir.port,m->cfg.pin_dir.pin);
	}
	if(m->flag.status_changed){
		m->flag.status_changed = RESET;
		if(m->Status == motor_disable){
			m->Status = motor_enable;
		}
		else{
			m->Status = motor_disable;
		}
	}
	if(m->flag.start_pending){
		m->flag.start_pending = RESET;
		if(m->trap.state == ts_idle){
			m->trap.flag.startCMD = SET;
		}
	}
	if(m->flag.stop_pending){
		m->flag.stop_pending = RESET;
		if(m->trap.state != ts_idle){
			m->trap.flag.stopCMD = SET;
		}
	}
	if(m->flag.fault_latched){
		m->flag.fault_latched = RESET;
		// چک کردن خطا
	}
	
}
//
motor_status_t motor_get_status(motor_t *m){
	return m->Status;
}
motor_state_t motor_get_state(motor_t *m){
	return m->state;
}
motor_error_t motor_get_error(motor_t *m){
	return m->error;
}
motor_pos_t motor_get_position(motor_t *m){
	return m->pos;
}
//
void motor_clear_fault(motor_t *m){
	
}
void start_motor_lowSpeed(motor_t *m){
	m->startMode = MOTOR_START_LOW_SPEED;
	uint16_t ccr = (0.5f * __HAL_TIM_GetAutoreload(m->trap.timer));
	__HAL_TIM_SetCompare(m->trap.timer,m->trap.channel,ccr);
	HAL_GPIO_WritePin(m->cfg.pin_sleep.port,m->cfg.pin_sleep.pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(m->cfg.pin_dir.port,m->cfg.pin_dir.pin,GPIO_PIN_SET);
	HAL_TIM_PWM_Start(m->trap.timer,m->trap.channel);
}
void motor_handler(motor_t *m){
	motor_update(m,HAL_GetTick());
	
	switch(m->startMode){
		case MOTOR_START_LOW_SPEED:{
			if(m->flag.captureDirEdge){
				m->flag.captureDirEdge = 0;
				m->startMode = MOTOR_START_TRAPEZOIDAL;
			}
			else if(m->flag.captureDirACT){
				m->flag.captureDirACT = 0;
				m->startMode = MOTOR_START_TRAPEZOIDAL;
			}
			break;
		}
		case MOTOR_START_TRAPEZOIDAL:{
			if(m->pos == motor_pos_close){
				HAL_GPIO_WritePin(m->cfg.pin_dir.port,m->cfg.pin_dir.pin,(m->cfg.pin_dir.active_lvl & 0x01));
			}
			else if(m->pos == motor_pos_open){
				HAL_GPIO_WritePin(m->cfg.pin_dir.port,m->cfg.pin_dir.pin,(~m->cfg.pin_dir.active_lvl & 0x01));
			}
			else{
				m->pos = motor_pos_float;
			}
			trapezoidal_stateMachine(&m->trap);
			break;
		}
		case MOTOR_START_NORMAL:{
			
			break;
		}
	}
	/*switch(m->pos){
		case motor_pos_close:{
			
			break;
		}
		case motor_pos_float:{
			
			break;
		}
		case motor_pos_open:{
			
			break;
		}
	}*/
	
}