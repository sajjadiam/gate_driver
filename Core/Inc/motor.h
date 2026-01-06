#ifndef MOTOR_H
#define MOTOR_H
#include "trapezoidal.h"

typedef enum{ // direction
	MOTOR_DIR_CW = 0,
	MOTOR_DIR_CCW ,
}motor_dir_t;

typedef enum{ // state
	MOTOR_STATE_IDLE = 0,
	MOTOR_STATE_RUNNING,
	MOTOR_STATE_STOPPING,
	MOTOR_STATE_FAULT,
}motor_state_t;

typedef enum{ // start mode
	MOTOR_START_LOW_SPEED	= 0,
	MOTOR_START_TRAPEZOIDAL, 
	MOTOR_START_NORMAL,
}motor_start_mode_t;

typedef enum{ // stop mode
	MOTOR_STOP_COAST	= 0,
	MOTOR_STOP_BRAKE, 
	MOTOR_STOP_SOFT,
}motor_stop_mode_t;

typedef enum{ // error
	MOTOR_ERR_NONE = 0,
	MOTOR_ERR_NFAULT,
	MOTOR_ERR_OVERCURRENT,
	MOTOR_ERR_TIMEOUT,
	MOTOR_ERR_LIMIT,        // به لیمیت خورد
} motor_error_t;

typedef enum{ // status
	motor_disable					= 0,
	motor_enable	 ,
}motor_status_t;

typedef struct{ // motor pins
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t active_lvl; 
} motor_pin_t;

typedef struct{ // motor config
    motor_pin_t pin_dir;     // جهت ph
    motor_pin_t pin_sleep;   // NSLEEP (اختیاری)
    motor_pin_t pin_nfault;  // NFAULT input (اختیاری)
    uint32_t move_timeout_ms;    // تایم‌اوت حرکت
} motor_cfg_t;

typedef struct{ // motor flag
	uint8_t captureDirEdge: 1;
	uint8_t captureDirACT : 1;
	uint8_t start_pending : 1;
	uint8_t stop_pending  : 1;
	uint8_t	dir_changed		: 1;
	uint8_t	status_changed: 1;
	uint8_t fault_latched : 1;
} motor_flags_t;

typedef enum{ // motor position
	motor_pos_close		= 0,
	motor_pos_float,
	motor_pos_open,
}motor_pos_t;

typedef struct{ // motor structure
	motor_cfg_t					cfg;
	trapezoidal_t 			trap;
	uint32_t 						start_tick_ms;
	motor_flags_t				flag;
	motor_start_mode_t	startMode;
	motor_stop_mode_t		stopMode;
	motor_state_t				state;
	motor_dir_t					dir;
	motor_status_t			Status;
	motor_pos_t					pos;
	motor_error_t				error;
}motor_t;



//init
void motor_init_pins(motor_t *m, motor_cfg_t *cfg);
void motor_init_timer(motor_t *m, TIM_HandleTypeDef* htim,uint32_t channel, uint8_t pwmMaxPrecent);
//set activity
void motor_activity(motor_t *m, motor_status_t status);
void motor_set_dir(motor_t *m, motor_dir_t dir);
void motor_start(motor_t *m, motor_start_mode_t mode);
void motor_stop(motor_t *m, motor_stop_mode_t mode);
//check response
void motor_update(motor_t *m, uint32_t now_ms);     // هر tick صدا بزن
//
motor_status_t motor_get_status(motor_t *m);
motor_state_t motor_get_state(motor_t *m);
motor_error_t motor_get_error(motor_t *m);
motor_pos_t motor_get_position(motor_t *m);
//
void motor_clear_fault(motor_t *m);
//
void motor_handler(motor_t *m);
void start_motor_lowSpeed(motor_t *m);
#endif//MOTOR_H 