#include "ntc.h"
#include <math.h>

void ntc_init(ntc_t* s, const ntc_hw_t* hw, const ntc_model_params_t* mp, const ntc_filter_t* filt){
	*s = (ntc_t){0};
	s->hw = *hw;
	s->mp = *mp;
	s->filt = *filt;
	if (s->filt.alpha < 0.0f) s->filt.alpha = 0.0f;
	if (s->filt.alpha > 1.0f) s->filt.alpha = 1.0f;
	s->valid = false;
	s->err = NTC_ERR_NONE;
	s->adc_ratio_f = NAN;
	s->T_C = NAN;
	s->T_K = NAN;
}
static float ntc_calc_R(const ntc_hw_t* hw,float ratio, NTC_ERROR* err){
	if (!hw) {
		if(err) {
			*err |= NTC_ERR_MODEL;
		}
		return NAN;
	}
	if (!err) {
		// اگر err ندادی، حداقل کرش نکنیم
		static NTC_ERROR dummy;
		err = &dummy;
	}
	// اگر R_fixed معتبر نیست
	if (!(hw->R_fixed_ohm > 0.0f)) {
		*err |= NTC_ERR_MODEL;
		return NAN;
	}
	// جلوگیری از تقسیم بر صفر/نزدیک صفر
	// (اگر min/max درست تنظیم شده باشه، این هم عملاً پوشش داده می‌شه)
	const float eps = 1e-6f; // 1 count فاصله برای 12bit کافی است
	if (!isfinite(ratio) || ratio <= eps || ratio >= (1.0f - eps)) {
		*err |= NTC_ERR_DIV0;   // عملاً یعنی نسبت نامعتبر/تقسیم بر صفر
		return NAN;
	}
	if (hw->topology == NTC_TO_GND) {
		return hw->R_fixed_ohm * (ratio / (1.0f - ratio));
	} 
	else{
		return hw->R_fixed_ohm * ((1.0f - ratio) / ratio);
	}
}
static float ntc_calc_TK(const ntc_model_params_t* mp, float R_ntc, NTC_ERROR* err){
	if (!(R_ntc > 0.0f)) {
		*err |= NTC_ERR_LOG;
		return NAN;
	}

	if (mp->model == NTC_MODEL_BETA){
		const float R0 = mp->p.beta.R0_ohm;
		const float T0 = mp->p.beta.T0_K;
		const float B  = mp->p.beta.beta_K;
		if (!(R0 > 0.0f) || !(T0 > 0.0f) || !(B > 0.0f)) {
			*err |= NTC_ERR_MODEL;
			return NAN;
		}
		const float lnRR0 = logf(R_ntc / R0);
		// 1/T = 1/T0 + (1/B)*ln(R/R0)
		const float invT = (1.0f / T0) + (lnRR0 / B);
		if(!(invT > 0.0f)){
			*err |= NTC_ERR_MODEL;
			return NAN;
		}
		return 1.0f / invT;
	} 
	else{ // Steinhart-Hart
		const float A = mp->p.sh.A;
		const float B = mp->p.sh.B;
		const float C = mp->p.sh.C;
		const float lnR = logf(R_ntc);
		// 1/T = A + B * ln(R) + C * ln(R) * ln(R) * ln(R)
		const float invT = A + B*lnR + C*lnR*lnR*lnR;
		if(!(invT > 0.0f)){
			*err |= NTC_ERR_MODEL;
			return NAN;
		}
		return 1.0f / invT;
	}
}
void ntc_update_adc(ntc_t* s, uint16_t adc_raw){
	s->adc_raw = adc_raw;
	s->err = NTC_ERR_NONE;
	s->valid = false;
	//
	if(adc_raw < s->hw.adc_min_valid || adc_raw > s->hw.adc_max_valid){
		s->err |= NTC_ERR_ADC_OOR;
		return;
	}
	if(s->hw.adc_fullscale == 0u){
		s->err |= NTC_ERR_MODEL;
		return;
	}
	//
	float ratio = (float)adc_raw / (float)s->hw.adc_fullscale;
	if(ratio < 0.0f){
		ratio = 0.0f;
	}
	if(ratio > 1.0f){
		ratio = 1.0f;
	}
	//
	s->adc_ratio = ratio;
	// Filtering
	if(s->filt.filter_on_temp == false){
		if(!isfinite(s->adc_ratio_f)){
			s->adc_ratio_f = ratio;
		}
		s->adc_ratio_f += s->filt.alpha * (ratio - s->adc_ratio_f);
	}
	else{
		// We'll filter after temperature is computed
		s->adc_ratio_f = ratio;
	}
	// Convert to resistance
	s->R_ntc_ohm = ntc_calc_R(&s->hw, s->adc_ratio_f, &s->err);
	if(!isfinite(s->R_ntc_ohm)){
		return;
	}
	// Convert to temperature
	s->T_K = ntc_calc_TK(&s->mp, s->R_ntc_ohm, &s->err);
	if(!isfinite(s->T_K)){
		return;
	}
	float T_C = s->T_K - 273.15f;
	// Optional filtering on temperature
	if (s->filt.filter_on_temp) {
		if (!isfinite(s->T_C)){
			s->T_C = T_C;
		}
		s->T_C += s->filt.alpha * (T_C - s->T_C);
		s->T_K = s->T_C + 273.15f;
	} 
	else{
		s->T_C = T_C;
	}
	//
	s->valid = (s->err == NTC_ERR_NONE);
}