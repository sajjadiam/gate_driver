#include "trapezoidal.h"



void trapezoidal_func_init(trapezoidal_t* trap , TIM_HandleTypeDef* htim,uint32_t channel ,uint8_t pwmMaxPrecent){
	trap->timer 					= htim;
	trap->channel 				= channel;
	trap->vMax 						= pwmMaxPrecent;
	trap->state 					= ts_idle;
}
static void trapezoidal_func_idle(trapezoidal_t* trap){
	// 1. frist in
	if(!trap->flag.idleFristIn){
		trap->flag.idleFristIn = SET;
		HAL_TIM_PWM_Stop(trap->timer,trap->channel);
	}
	// 2. Repeat loop
	// get start cmd
	// 3. go next Condition
	if(trap->flag.startCMD){
		trap->flag.startCMD = RESET;
		trap->flag.idleFristIn = RESET;
		trap->state = ts_rampUp;
	}
}
static void trapezoidal_func_rampUp(trapezoidal_t* trap){
	// 1. check stop flag
	if(trap->flag.stopCMD){
		trap->flag.stopCMD = RESET;
		trap->state = ts_idle;
	}
	// 2. frist in
	if(!trap->flag.rampUpFristIn){
		trap->flag.rampUpFristIn = SET;
		trap->couter = RESET;
		__HAL_TIM_SetCompare(trap->timer,trap->channel,RESET);
		HAL_TIM_PWM_Start(trap->timer,trap->channel);
	}
	// 3. Repeat loop
	trap->couter++;
	uint16_t pwmPercentage = (trap->couter * (uint16_t)trap->vMax / RAMP_UP_TIME) ; // (counter / max counter) * max pwm = pwmPercentage
	if(pwmPercentage > (uint16_t)trap->vMax){
		pwmPercentage = (uint16_t)trap->vMax;
	}
	uint16_t arr = __HAL_TIM_GetAutoreload(trap->timer);
	uint16_t ccr = pwmPercentage * arr;
	if(ccr > arr){
		ccr = arr;
	}
	__HAL_TIM_SetCompare(trap->timer,trap->channel,ccr);
	
	// 4. go next Condition 
	if(trap->couter >= RAMP_UP_TIME){
		trap->flag.rampUpFristIn = RESET;
		trap->state 					= ts_hold;
	}
}
static void trapezoidal_func_hold(trapezoidal_t* trap){
	// 1. check stop flag
	if(trap->flag.stopCMD){
		trap->flag.stopCMD = RESET;
		trap->state = ts_idle;
	}
	// 2. frist in
	if(!trap->flag.holdFristIn){
		trap->flag.holdFristIn = SET;
		trap->couter = RESET;
		uint16_t arr = __HAL_TIM_GetAutoreload(trap->timer);
		uint16_t ccr = trap->vMax * arr;
		if(ccr > arr){
			ccr = arr;
		}
		__HAL_TIM_SetCompare(trap->timer,trap->channel,ccr);
	}
	// 3. Repeat loop
	trap->couter++;
	
	// 4. go next Condition  
	if(trap->couter >= HOLD_TIME){
		trap->flag.stopCMD 	= SET;
		trap->state 				= ts_rampDown;
	}
}
static void trapezoidal_func_rampDown(trapezoidal_t* trap){
	// 1. check stop flag
	if(trap->flag.stopCMD){
		trap->flag.stopCMD = RESET;
		trap->state = ts_idle;
	}
	// 2. frist in
	if(!trap->flag.rampDownFristIn){
		trap->flag.rampDownFristIn = SET;
		trap->couter = RESET;
	}
	// 3. Repeat loop
	trap->couter++;
	float pwmPercentage = ((float)trap->couter * trap->vMax / RAMP_UP_TIME) ; // (counter / max counter) * max pwm = pwmPercentage
	uint16_t arr = __HAL_TIM_GetAutoreload(trap->timer);
	uint16_t ccr = pwmPercentage * arr;
	__HAL_TIM_SetCompare(trap->timer,trap->channel,ccr);
	// 4. go next Condition 
	if(trap->couter >= RAMP_DOWN_TIME){
		trap->flag.rampDownFristIn = RESET;
		trap->state 					= ts_idle;
	}
}




typedef void (*trapezoidal_func)(trapezoidal_t* trap);
static trapezoidal_func trap_func_arr[ts_END] = {
	[ts_idle]			= trapezoidal_func_idle,
	[ts_rampUp]		= trapezoidal_func_rampUp,
	[ts_hold]			= trapezoidal_func_hold,
	[ts_rampDown]	= trapezoidal_func_rampDown
};
void trapezoidal_stateMachine(trapezoidal_t* trap){
	trap_func_arr[trap->state](trap);
}