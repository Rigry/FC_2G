#pragma once

#include "interrupt.h"
#include "uart.h"
#include "ntc.h"

constexpr float k_adc   = 3.3 / 4095;
constexpr float k_adc_i = 3 * k_adc / 2 / 0.0167; // 3 и 2 потому что делитель 10 и 20 кОм, 0,025 В/А
constexpr float k_u     = 3.29 * 450.00 / 4095.00;

const uint16_t measure_time{25};
constexpr uint16_t qty_measure = 200 / measure_time;

class Service
{
	ADC_& adc;
	NTC& ntc;

	uint16_t arr_new_hv[qty_measure] { 0 };
	uint8_t m { 0 };
	int16_t HV_avarage { 0 };

	Timer timer;
	Timer measure_timer { measure_time };

	float new_hv{0};

public:

	uint16_t voltage_board{0};
	uint16_t high_voltage{0};
	uint16_t convertor_temp{0};
	uint8_t current{0};
	uint8_t current_drive{0};

	Service (
		  ADC_& adc
		, NTC& ntc
	) : adc              {adc}
	  , ntc              {ntc}
	{}

	void operator()(){

		voltage_board  = k_adc * adc[V24] * 100 + 3; // 3 падение на диоде
		convertor_temp  = ntc(adc[Trad]);
		current        = adc.s;
		current_drive      = adc.current();

		new_hv = (adc.value_HV() * 352 / 4095 * 42) / 10;
		if (measure_timer.done()) {
			measure_timer.stop();
			measure_timer.start();
			arr_new_hv[m] = new_hv;
			if (m < (qty_measure - 1))
				m++;
			else
				m = 0;
			HV_avarage = 0;
			for (auto i = 0; i < qty_measure; i++) {
				HV_avarage += arr_new_hv[i];
			}
			HV_avarage /= qty_measure;
			high_voltage += (HV_avarage - high_voltage) * 10 / 40;
		}
	}

};
