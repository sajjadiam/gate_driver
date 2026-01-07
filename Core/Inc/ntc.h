#ifndef NTC_H
#define NTC_H

#include <stdint.h>
#include <stdbool.h>
typedef enum {
	NTC_TO_GND = 0,   // Pull-up resistor to Vref, NTC to GND, ADC at middle node
	NTC_TO_VREF = 1   // Pull-down resistor to GND, NTC to Vref, ADC at middle node
} ntc_topology_t;

typedef enum {
	NTC_MODEL_BETA = 0,
	NTC_MODEL_STEINHART_HART = 1
} ntc_model_t;

typedef struct {
    // Beta model parameters
    float R0_ohm;     // nominal resistance at T0
    float T0_K;       // nominal temperature in Kelvin (e.g. 298.15)
    float beta_K;     // beta constant in Kelvin
} ntc_beta_t;

typedef struct {
    // Steinhart-Hart parameters
    float A;
    float B;
    float C;
} ntc_sh_t;

typedef struct {  // ntc hardware
    // Electrical configuration
    ntc_topology_t topology;
    uint32_t adc_fullscale;  // e.g. 4095 for 12-bit
    float vref_V;            // ADC reference (or analog supply) in volts
    float R_fixed_ohm;       // series / pull-up/down resistor value in ohms

    // Optional: ADC sampling sanity range (detect open/short)
    uint32_t adc_min_valid;  // inclusive
    uint32_t adc_max_valid;  // inclusive
} ntc_hw_t;

typedef struct { // model
    ntc_model_t model;
    union {
        ntc_beta_t beta;
        ntc_sh_t   sh;
    } p;
} ntc_model_params_t;

typedef struct { // filter
    // Simple IIR filter on ADC or temperature
    // y += alpha*(x - y)
    float alpha;       // 0..1 , alpha=1 => no filtering
    bool  filter_on_temp; // true: filter temperature, false: filter ADC ratio
} ntc_filter_t;
typedef enum {
	NTC_ERR_NONE     		= 0,
	NTC_ERR_ADC_OOR  		= 1u << 0,  // ADC out of valid range
	NTC_ERR_DIV0     		= 1u << 1,  // division by zero / invalid ratio
	NTC_ERR_LOG      		= 1u << 2,  // log domain error
	NTC_ERR_MODEL    		= 1u << 3,   // model config invalid
	
}NTC_ERROR;
typedef struct { // ntc
    // Public configuration
    ntc_hw_t hw;
    ntc_model_params_t mp;
    ntc_filter_t filt;

    // Runtime state / outputs
    bool  valid;
    uint32_t adc_raw;
    float adc_ratio;     // 0..1
    float adc_ratio_f;   // filtered
    float R_ntc_ohm;
    float T_K;
    float T_C;

    // Last error flags (bitmask)
    NTC_ERROR err;
} ntc_t;

/* ---------- Error bits ---------- */

/* ---------- API ---------- */

void ntc_init(ntc_t* s, const ntc_hw_t* hw, const ntc_model_params_t* mp, const ntc_filter_t* filt);
void ntc_update_adc(ntc_t* s, uint16_t adc_raw);






#endif//NTC_H