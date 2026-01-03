#ifndef TRAPEZOIDAL_H
#define TRAPEZOIDAL_H

#include <stdint.h>
#include "stm32f1xx_hal.h"


#define RAMP_UP_TIME			250
#define RAMP_DOWN_TIME		250
#define HOLD_TIME					100


typedef enum{
	ts_init			= 0,
	ts_idle,
	ts_rampUp,
	ts_hold,
	ts_rampDown,
}trapezoidal_state_t;
typedef struct{
	TIM_HandleTypeDef*	timer;
	uint32_t						channel;
	uint16_t						rampUpCouter;
	uint16_t						rampDownCouter;
	uint16_t						holdCouter;
	uint8_t 						vMax;			// In percentage of pwm 0 to 100
	trapezoidal_state_t	state;
}trapezoidal_t;


#endif//TRAPEZOIDAL_H