#include "trapezoidal.h"

static trapezoidal_t trap_profile = {0};

static void trapezoidal_func_init(trapezoidal_t* trap , TIM_HandleTypeDef* htim,uint32_t channel ,uint8_t pwmPrecernt){
	trap->timer 					= htim;
	trap->channel 				= channel;
	trap->holdCouter 			= 0;
	trap->rampDownCouter 	= 0;
	trap->rampUpCouter 		= 0;
	trap->vMax 						= pwmPrecernt;
	trap->state 					= ts_idle;
}
static void trapezoidal_func_idle(trapezoidal_t* trap){
	// frist in
	// every 
	// out 
}