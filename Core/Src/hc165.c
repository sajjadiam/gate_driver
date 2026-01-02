#include "hc165.h"
#include "main.h"

uint8_t hc165_out = 0xFF;
static int8_t hc165_bitCount = 0;
static hc165_ms_t hc165State = hc165_init;
typedef void(*hc165_Func)(void);

void hc165_clkTrig(void){
	HAL_GPIO_WritePin(HC165_CLK_GPIO_Port,HC165_CLK_Pin,GPIO_PIN_RESET);
}
void hc165_clkUnTrig(void){
	HAL_GPIO_WritePin(HC165_CLK_GPIO_Port,HC165_CLK_Pin,GPIO_PIN_SET);
}
void hc165_parallelTrig(void){
	HAL_GPIO_WritePin(HC165_PL_GPIO_Port,HC165_PL_Pin,GPIO_PIN_RESET);
}
void hc165_parallelLoadUnTrig(void){
	HAL_GPIO_WritePin(HC165_PL_GPIO_Port,HC165_PL_Pin,GPIO_PIN_SET);
}
uint8_t hc165_dataRead(void){
	return (HAL_GPIO_ReadPin(HC165_DATA_GPIO_Port,HC165_DATA_Pin) == GPIO_PIN_SET) ? 1 : 0;
}
static void hc165Init		(void){
	hc165_clkUnTrig();
	hc165_parallelLoadUnTrig();
	hc165State = hc165_startCapture;
}
static void startCapture	(void){
	hc165_parallelTrig();
	hc165State = hc165_endCapture;
}
static void endCapture		(void){
	hc165_parallelLoadUnTrig();
	hc165_bitCount = 7;
	hc165State = hc165_getSample;
}
static void getSample		(void){
	if(hc165_dataRead()){
		hc165_out |= 1 << hc165_bitCount;
	}
	else{
		hc165_out &= ~(1 << hc165_bitCount);
	}
	hc165_clkTrig();
	hc165State = hc165_trigClock;
}
static void trigClock		(void){
	hc165_clkUnTrig();
	if(--hc165_bitCount >= 0){
		hc165State = hc165_getSample;
	}
	else{
		hc165State = hc165_startCapture;
	}
}
static hc165_Func hc165MS[hc165_end] = {
	[hc165_init				 ] = hc165Init		,
	[hc165_startCapture] = startCapture	,
	[hc165_endCapture	 ] = endCapture		,
	[hc165_getSample	 ] = getSample		,
	[hc165_trigClock	 ] = trigClock		,
};
void hc165_stateMachine(void){
	hc165MS[hc165State]();
}