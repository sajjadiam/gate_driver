#ifndef TRAPEZOIDAL_H
#define TRAPEZOIDAL_H

#include <stdint.h>
#include "stm32f1xx_hal.h"


#define RAMP_UP_TIME			5000
#define RAMP_DOWN_TIME		5000
#define HOLD_TIME					5000
#define SET								1
#define RESET							0

typedef enum{
	ts_idle			= 0,
	ts_rampUp,
	ts_hold,
	ts_rampDown,
	ts_END,
}trapezoidal_state_t;
typedef struct{
	uint8_t idleFristIn			: 1;
	uint8_t startCMD				: 1;
	uint8_t stopCMD					: 1;
	uint8_t rampUpFristIn		: 1;
	uint8_t holdFristIn			: 1;
	uint8_t rampDownFristIn	: 1;
}trapFlags_t;
typedef struct{
	TIM_HandleTypeDef*	timer;
	uint32_t						channel;
	uint16_t						couter;
	uint8_t 						vMax;			// In percentage of pwm 0 to 100
	trapezoidal_state_t	state;
	trapFlags_t					flag;
}trapezoidal_t;

void trapezoidal_func_init(trapezoidal_t* trap , TIM_HandleTypeDef* htim,uint32_t channel ,uint8_t pwmPrecernt);
void trapezoidal_stateMachine(trapezoidal_t* trap);
#endif//TRAPEZOIDAL_H