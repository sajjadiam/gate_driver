#ifndef HC165_H
#define HC165_H

#include <stdint.h>

typedef enum{
	hc165_init   				= 0,
	hc165_startCapture,
	hc165_endCapture,
	hc165_getSample,
	hc165_trigClock,
	hc165_end,
}hc165_ms_t;
void hc165_clkTrig(void);
void hc165_clkUnTrig(void);
void hc165_parallelTrig(void);
void hc165_parallelLoadUnTrig(void);
uint8_t hc165_dataRead(void);

void hc165_stateMachine(void);
extern uint8_t hc165_out;
#endif //HC165_H