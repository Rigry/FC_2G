#pragma once

#include "interrupt.h"
#include <cmath>

const uint16_t qty_point{60};

enum {phase_B = 1, phase_C = 2, phase_A = 3, S = 4};
enum {V24 = 0, Trad = 1, HV = 2};

class ADC_ : TickSubscriber
{
	Interrupt& adc_callback;
	Interrupt& adc_injected_callback;

	uint8_t qty_channel{0};
	uint16_t time_refresh{1};

	uint16_t time{0};
	uint16_t buffer[3];

	int16_t arr_current_S[9]{0};
	int16_t arr_current_A[9]{0};
	int16_t arr_current_B[9]{0};
	int16_t arr_current_C[9]{0};
	uint16_t arr_HV[16]{0};
	uint16_t adc_HV{0};
	uint16_t h_voltage{0};
	uint16_t new_hv{0};

	int16_t new_s{0};

	int16_t r{0};
	int16_t new_r{0};

	uint8_t j{0};
	uint8_t m{0};

	bool work{false};
	bool measure{false};
	bool error{false};
	bool over_cur_s{false};
	bool over_cur_a{false};
	bool over_cur_b{false};
	bool over_cur_c{false};
	bool error_HV{false};
	bool Km_check{false};

	bool sense_s{false};
	bool sense_a{false};
	bool sense_b{false};
	bool sense_c{false};

	int16_t arr_S[9]{0};
	int16_t arr_A[9]{0};
	int16_t arr_B[9]{0};
	int16_t arr_C[9]{0};

	uint8_t error_a{0};
	uint8_t error_b{0};
	uint8_t error_c{0};

	uint16_t max_current{20};
	uint8_t over_current_s{0};
	uint8_t over_cur_phase{0};

	bool second_half {false};

	uint16_t max_current_phase{20};
	uint8_t over_current_a{0};
	uint8_t over_current_b{0};
	uint8_t over_current_c{0};

	void adc_interrupt() {
		HAL_ADC_Stop_DMA (&hadc1);
		if (new_hv > 2250) {  // 250 ~= 96V
			error_HV = true;
		}
		new_hv = buffer[HV];
		h_voltage = new_hv;
	}

	void adc_injected_interrupt() {

		HAL_ADCEx_InjectedStop_IT (&hadc2);
		arr_current_S[j] = HAL_ADCEx_InjectedGetValue(&hadc2, S);
		arr_current_A[j] = HAL_ADCEx_InjectedGetValue(&hadc2, phase_A);
		arr_current_B[j] = HAL_ADCEx_InjectedGetValue(&hadc2, phase_B);
		arr_current_C[j] = HAL_ADCEx_InjectedGetValue(&hadc2, phase_C);

		measure = true;

		if(not work) {
			offset_I_S = 0;
			offset_I_A = 0;
			offset_I_B = 0;
			offset_I_C = 0;
			for (auto i = 0; i < 9; i++) {
				offset_I_S += arr_current_S[i];
				offset_I_A += arr_current_A[i];
				offset_I_B += arr_current_B[i];
				offset_I_C += arr_current_C[i];
			}
			offset_I_S /= (9);
			sense_s = offset_I_S > 2200 or offset_I_S < 2000;
			offset_I_A /= (9);
			sense_a = offset_I_A > 2200 or offset_I_A < 2000;
			offset_I_B /= (9);
			sense_b = offset_I_B > 2200 or offset_I_B < 2000;
			offset_I_C /= (9);
			sense_c = offset_I_C > 2200 or offset_I_C < 2000;


//			error_a = 0;
//			error_b = 0;
//			error_c = 0;

			over_current_s = 0;
			over_current_a = 0;
			over_current_b = 0;
			over_current_c = 0;

		} else if (work) {

			arr_S[j] = abs(arr_current_S[j] - offset_I_S);
			arr_A[j] = abs(arr_current_A[j] - offset_I_A);
			arr_B[j] = abs(arr_current_B[j] - offset_I_B);
			arr_C[j] = abs(arr_current_C[j] - offset_I_C);

			new_s = arr_S[j] * 10 / 33;
			a = (arr_current_A[j] - offset_I_A) * 10 / 33;
			b = (arr_current_B[j] - offset_I_B) * 10 / 33;
			c = (arr_current_C[j] - offset_I_C) * 10 / 33;

			s += (new_s - s) / 8;

//			if (arr_A[j] <= 5 and Km_check) {
//				error_a++;
//				if (error_a > 6)
//					error = true;
//			}
//
//			if (arr_B[j] <= 5 and Km_check) {
//				error_b++;
//				if (error_b > 6)
//					error = true;
//			}
//
//			if (arr_C[j] <= 5 and Km_check) {
//				error_c++;
//				if (error_c > 6)
//					error = true;
//			}

			if (arr_S[j] / 33 >= max_current and Km_check) {
				over_current_s++;
				if (over_current_s >= 2)
					over_cur_s = true;
			}

			if (arr_A[j] / 33 >= max_current_phase and Km_check) {
				over_current_a++;
				if (over_current_a >= 3) {
					over_current_a = 0;
					over_cur_a = true;
				}
			}

			if (arr_B[j] / 33 >= max_current_phase and Km_check) {
				over_current_b++;
				if (over_current_b >= 3) {
					over_current_b = 0;
					over_cur_b = true;
				}
			}

			if (arr_C[j] / 33 >= max_current_phase and Km_check) {
				over_current_c++;
				if (over_current_c >= 3) {
					over_current_c = 0;
					over_cur_c = true;
				}
			}

			new_r = ( (std::sqrt( std::pow((a - b / 2 - c / 2), 2) + std::pow( (b * 17 / 20 - c * 17 / 20), 2) ) * 2) / 3);

			r += (new_r - r) / 4;
		}

		if (j < 8) j++;
		else {
			j = 0;
//			error_a = 0;
//			error_b = 0;
//			error_c = 0;
			over_current_s = 0;
			over_current_a = 0;
			over_current_b = 0;
			over_current_c = 0;
		}
	}

	using Parent = ADC_;

	struct ADC_interrupt : Interrupting {
		Parent &parent;
		ADC_interrupt(Parent &parent) :
				parent(parent) {
			parent.adc_callback.subscribe(this);
		}
		void interrupt() override {
			parent.adc_interrupt();
		}
	} adc_ { *this };

	struct ADC_INJ_interrupt : Interrupting {
		Parent &parent;
		ADC_INJ_interrupt(Parent &parent) :
				parent(parent) {
			parent.adc_injected_callback.subscribe(this);
		}
		void interrupt() override {
			parent.adc_injected_interrupt();
		}
	} adc_injected_ { *this };

public:

	ADC_(Interrupt& adc_callback, Interrupt& adc_injected_callback, uint8_t qty_channel, uint16_t time_refresh)
    : adc_callback {adc_callback}
    , adc_injected_callback {adc_injected_callback}
    , qty_channel  {qty_channel}
    , time_refresh {time_refresh}
	{
		subscribed = false;
		if (time_refresh > 0)
		  subscribe();
	}

	int16_t offset_I_S{0};
	int16_t offset_I_A{0};
	int16_t offset_I_B{0};
	int16_t offset_I_C{0};
	int16_t s{0};
	int16_t a{0};
	int16_t b{0};
	int16_t c{0};

	void measure_offset() {
		work = false;
	}

	void measure_value() {
		work = true;
	}

	bool error_sense_s() {
		return sense_s;
	}

	bool error_sense_a() {
		return sense_a;
	}

	bool error_sense_b() {
		return sense_b;
	}

	bool error_sense_c() {
		return sense_c;
	}

	uint16_t operator[](uint8_t i) {
		return buffer[i];
	}

	uint16_t value(uint8_t i) {
		if (i == S)
			return arr_current_S[j];
		if (i == phase_A)
			return arr_current_A[j];
		if (i == phase_B)
			return arr_current_B[j];
		if (i == phase_C)
			return arr_current_C[j];
		return 0;
//		if (i == HV)
//			return h_voltage;
	}

	uint16_t current(){
		return r;
	}

	uint16_t value_HV() {
		return h_voltage;
	}

	bool is_measure() { return measure; }
	void reset_measure() { measure = false; }
	bool is_error(){return error;}
	void reset_error(){error = false;}
	bool is_over_s(){return over_cur_s;}
	void reset_over_s(){over_cur_s = false;}
	bool is_over_a(){return over_cur_a;}
	void reset_over_a(){over_cur_a = false;}
	bool is_over_b(){return over_cur_b;}
	void reset_over_b(){over_cur_b = false;}
	bool is_over_c(){return over_cur_c;}
	void reset_over_c(){over_cur_c = false;}
	bool is_error_HV(){return error_HV;}
	void reset_error_HV(){error_HV = false;}
	void what_Km(uint16_t k) {Km_check = k > 50 ? true : false;}

	void notify(){
		if (time++ >= time_refresh) {
		   time = 0;
		   HAL_ADC_Start_DMA(&hadc1, (uint32_t*)buffer, qty_channel);
		}
		if(not time % 100 and not work)
			HAL_ADCEx_InjectedStart_IT(&hadc2);
	}

	void set_max_current(uint16_t v){
		max_current = v;
	}

	void set_max_current_phase(uint16_t v){
		max_current_phase = v;
	}


};

Interrupt adc_callback;
Interrupt adc_injected_callback;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc){
	if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
	{
		adc_callback.interrupt();
	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC2) //check if the interrupt comes from ACD2
	{
		adc_injected_callback.interrupt();
	}
}

