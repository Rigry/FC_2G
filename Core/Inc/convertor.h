#pragma once

#include "adc.h"
#include "service.h"
#include "contactor.h"
#include "interrupt.h"
#include "pin.h"

class Convertor {

	enum {SYNCHRON = false, ASYNCHRON = true};
	enum State {wait, starting} state{wait};

	ADC_& adc;
	Service<In_data, Out_data>& service;
	Contactor& contactor;
	Interrupt& period_callback;
//	Interrupt& adc_comparator_callback;
	Pin& led_red;
	Pin& ventilator;
	Pin& unload;
	Pin& TD_DM;
	Pin& Start;
	Pin& Motor;
	Pin& state_fc;
	Pin& reset_error;
	Pin& er_total;

	Timer timer;
	Timer rerun;
	Timer timer_stop;
	Timer clump_timer;

	const uint16_t sin_table[qty_point]{ 6000,  7054,  8029,  8917,  9708, 10392, 10963, 11412, 11737, 11934
									  , 11999, 11934, 11737, 11412, 10963, 10392, 10963, 11412, 11737, 11934
									  , 11999, 11934, 11737, 11412, 10963, 10392,  9708,  8917,  8029,  7054
									  ,  6000,  4880,  3708,  2495,  1255,     0,     0,     0,     0,     0
									  ,     0,     0,     0,     0,     0,     0,     0,     0,     0,     0
									  ,     0,     0,     0,     0,     0,     0,  1255,  2495,  3708,  4880
									  };

	uint8_t r{0};
	uint8_t k{0};
	uint8_t m{20};
	uint8_t n{40};
	uint32_t Km{5};
	uint16_t Kp{1150};
	uint16_t frequency{100};
	uint16_t max_current{25};
	uint16_t need_current{4000};
	int16_t e{0};
	uint16_t time{2};
	uint16_t U_phase{0};
	bool U_stop{false};
	bool motor{false};
	uint16_t U_phase_max{0};
	uint8_t offset{25};
	uint8_t error{0};
	uint8_t error_S{0};
	uint8_t error_A{0};
	uint8_t error_C{0};
	uint8_t error_F{0};
	uint16_t min_ARR{360};
	uint16_t value_ARR{380};
	uint16_t ARR_ASIN{1999};

	bool enable{true};
	bool phase{false};
	bool cool{false};
	bool cold{false};
	bool switcher{false};

//	float radian = 10 * 3.14 / 180;
	uint32_t div_f = 6'000'000 / (qty_point);

	using Parent = Convertor;

	struct TIM3_interrupt: Interrupting {
		Parent &parent;
		TIM3_interrupt(Parent &parent) :
				parent(parent) {
			parent.period_callback.subscribe(this);
		}
		void interrupt() override {
			parent.period_interrupt();
		}
	} tim3_interrupt { *this };

//	struct adc_comparator_interrupt: Interrupting {
//		Parent &parent;
//		adc_comparator_interrupt(Parent &parent) :
//				parent(parent) {
//			parent.adc_comparator_callback.subscribe(this);
//		}
//		void interrupt() override {
//			parent.comparator_interrupt();
//		}
//	} adc_comparator_ { *this };

	void period_interrupt(){

//		condens = 1;

		if (Km >= 990) {
			Km = 990;
		}

		TIM1->CCR1 = Km * sin_table[m++] / 1000;
		TIM1->CCR2 = Km * sin_table[k++] / 1000;
		TIM1->CCR3 = Km * sin_table[n++] / 1000;

		if (k >= qty_point) {k = 0;}
		if (m >= qty_point) {m = 0;}
		if (n >= qty_point) {n = 0;}

		switcher ^= 1;

		if(switcher) HAL_ADCEx_InjectedStart_IT(&hadc2);

//		condens = 0;

	}

//	void comparator_interrupt() {
//
//	}

public:

	Convertor(ADC_& adc, Service<In_data, Out_data>& service, Contactor& contactor, Interrupt& period_callback/*, Interrupt& adc_comparator_callback*/
			, Pin& led_red,  Pin& ventilator, Pin& unload, Pin& TD_DM,Pin& Start, Pin& Motor, Pin& state_fc, Pin& reset_error, Pin& er_total)
	: adc{adc}, service{service}, contactor{contactor}, period_callback{period_callback}/*, adc_comparator_callback{adc_comparator_callback}*/
	, led_red{led_red}, ventilator{ventilator}, unload{unload}, TD_DM{TD_DM}, Start{Start}, Motor{Motor}, state_fc{state_fc}, reset_error{reset_error}, er_total{er_total}
	{rerun.time_set = 0; timer_stop.time_set = 0; clump_timer.time_set = 0;
		if(motor == SYNCHRON) {
			unload = true;
			clump_timer.start(3000);
		}
		motor = Motor;
		reset_error = false;
	}

	void operator() (){

//		condens = 1;
		service();

		service.outData.PWM = Km;
		service.outData.error.on = Start;
		service.outData.U_phase = U_phase;
		service.outData.error.overheat_c = not bool(TD_DM);
		service.outData.error.HV_low = U_stop;
		service.outData.error.voltage_board_low = (service.outData.voltage_board < 180);
		service.outData.error.voltage_board_high = (service.outData.voltage_board > 300);

		service.outData.max_current_A = min_ARR;
		service.outData.max_current_C = U_phase_max;
		service.outData.current_C = Kp;
		service.outData.max_current = TIM3->ARR;

		if(service.outData.high_voltage <= 300) U_stop = true;
		else if(service.outData.high_voltage > 310) {U_stop = false; adc.reset_error_HV();}

		if (service.outData.error.overheat_fc |= service.outData.convertor_temp >= 80) {
			service.outData.error.overheat_fc = service.outData.convertor_temp >= 70;
		}

/////////////////CONDITIONER
//		if (cool |= service.outData.convertor_temp >= 40) {
//			cool = service.outData.convertor_temp >= 30;
//		}
//
//		if(enable)
//			ventilator = cool;
//		else
//			ventilator = false;
/////////////////CONDITIONER

		if(contactor.is_on() and enable) alarm();
//		condens = 0;

		switch(state) {
		case wait:
//			motor = Motor;

if(motor == ASYNCHRON) {
/////////////////CONDITIONER
/*
	adc.set_max_current(35);
	adc.set_max_current_phase(36);
	if (service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
		min_ARR = (div_f / ((U_phase_max) * 9)) * 22; // 5/22 = 50/220
	} else {
		U_phase_max = 220;
		min_ARR = 1100;
	}
*/
/////////////////CONDITIONER
	adc.set_max_current(16);
	adc.set_max_current_phase(8);
	unload = false;
	if (service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
		min_ARR = (div_f / ((U_phase_max) * 5)) * 22; // 5/22 = 50/220
		if(min_ARR <= ARR_ASIN) min_ARR = ARR_ASIN;
	} else {
		U_phase_max = 110;
		min_ARR = ARR_ASIN;
	}


} else if (motor == SYNCHRON) {

	adc.set_max_current(16);
	adc.set_max_current_phase(20);
	if(clump_timer.done()) {
		clump_timer.stop(); unload = false;
	}
 	if(service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 940) / 141) * 115) / 100;
		min_ARR = ( (div_f / (U_phase_max)) * 50) / 70; // 70/53 = 280/212
		if(min_ARR < value_ARR) min_ARR = value_ARR;
	} else {
		U_phase_max = 212;
		min_ARR = value_ARR;
	}

}

			enable = Start and not rerun.isCount()
					 and not service.outData.error.overheat_fc and not service.outData.error.overheat_c
					 and not service.outData.error.HV_low
					 and not service.outData.error.voltage_board_low and not service.outData.error.voltage_board_high
					 and not U_stop and not er_total;

			if(rerun.done()) rerun.stop();

			if (enable){
				rerun.stop();
				contactor.on();
				if(contactor.is_on()) {
					pusk();
					state = State::starting;
				}
			}

			if (not Start) {
				rerun.stop();
				rerun.time_set = 0;
				led_red = false;
				adc.reset_error();
				phase = false;
			}

			break;
		case starting:

			adc.what_Km(Km);

if(motor == ASYNCHRON) {

/////////////////CONDITIONER

	/*
	if (service.outData.high_voltage > 400 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
		min_ARR = (div_f / ((U_phase_max) * 9)) * 22; // 5/22 = 50/220
	} else {
		U_phase_max = 220;
		min_ARR = 1100;
	}

	U_phase = ((((service.outData.high_voltage / 20) * Km) / 141) * 112) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
	Km = offset + ((Kp * (div_f / TIM3->ARR) / (service.outData.high_voltage)));

	if (TIM3->ARR <= (min_ARR + 10)) {
		error = 0;
	}

	if (Kp > 12000) {
		Kp = 12000;
	}

	if (TIM3->ARR <= min_ARR) {
		if (U_phase - U_phase_max > 10) {
			Kp--;
		} else {
			if((U_phase_max - U_phase > 10))
				Kp++;
		}

		if (adc.current() > 200) {
			if (Kp > 5000) {
				Kp -= 4;
			}
		}
	}

	if (adc.current() < 35) {
		if (Kp < 12000) {
			Kp++;
		}
	}

	if (TIM3->ARR > (min_ARR + 5)) {
		if (adc.current() > 75) {
			if (Kp >= 6000) {
				Kp--;
			}
		}
	}
*/
/////////////////CONDITIONER
//	service.outData.high_voltage = 550;

	if (service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
		U_phase_max = ((((service.outData.high_voltage / 20) * 990) / 141) * 115) / 100;
		min_ARR = (div_f / ((U_phase_max) * 5)) * 22; // 5/22 = 50/220
		if(min_ARR <= ARR_ASIN) min_ARR = ARR_ASIN;
	} else {
		U_phase_max = 110;
		min_ARR = ARR_ASIN;
	}

	U_phase = ((((service.outData.high_voltage / 20) * Km) / 141) * 112) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
	Km = offset + (Kp * (div_f / TIM3->ARR) / (service.outData.high_voltage + 1) );

	if (TIM3->ARR <= uint32_t(min_ARR + 5)) {
		unload = true;
		error = 0;
	}

	if (Kp > 10200) {
		Kp = 10200;
	}

	if (TIM3->ARR <= min_ARR) {
//		if (U_phase - U_phase_max > 10) {
//			Kp--;
//		} else {
//			if(adc.current() < 120 and (U_phase_max - U_phase > 10))
//			Kp++;
//		}

		if (adc.current() > 160) {
			if (Kp > 8000) {
				Kp -= 4;
			}
		}
	}

	if (adc.current() < 70) {
		if (Kp < 10000) {
			Kp++;
		}
	}

//	if (TIM3->ARR > uint32_t(min_ARR + 5)) {
//		if (adc.current() > 75) {
//			if (Kp >= 6000) {
//				Kp--;
//			}
//		}
//	}

	if (TIM3->ARR > uint32_t(min_ARR + 1)) {
			if (adc.current() > 42) {
				if (Kp >= 10000) {
					Kp--;
				}

				if (Kp <= 8000) {
					Kp++;
					Kp++;
				}
			}


		}


} else if(motor == SYNCHRON) {

//	service.outData.high_voltage = 550;

				if (service.outData.high_voltage > 300 and service.outData.high_voltage < 540) {
					U_phase_max = ((((service.outData.high_voltage / 20) * 980) / 141) * 115) / 100;
					min_ARR = ((div_f / (U_phase_max)) * 50) / 70; // 70/53 = 280/212
					if(min_ARR < value_ARR) min_ARR = value_ARR;
				} else {
					min_ARR = value_ARR;
					U_phase_max = 212;
				}

				U_phase = ((((service.outData.high_voltage / 20) * Km) / 141) * 115) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
//				U_phase += (U_phase_max - U_phase) * 10 / 50;
				Km = offset + Kp * (div_f / TIM3->ARR) / (service.outData.high_voltage);

				if (TIM3->ARR <= uint32_t(min_ARR + 5)) {
					unload = false;
					error = 0;
				}


//				if(TIM3->ARR <= min_ARR) {
//					if ((U_phase > U_phase_max)) {
//						if((U_phase - U_phase_max) > 8)
//							Kp--;
//					} else {
//						if ((U_phase_max - U_phase > 8))
//							Kp++;
//					}
//
//					if (adc.current() > 180) {
//						if (Kp > 1250) {
//							Kp -= 4;
//						}
//					}
//				}
				if (adc.current() < 35) {
					if (Kp < 2200) {
						Kp++;
					}
				}
				if (TIM3->ARR > uint32_t(min_ARR + 5)) {
					if (adc.current() > 110) {
						if(Kp > 1250) {
							Kp--;
						}
					}
				}

				if (Kp >= 2200) {
					Kp = 2200;
				}

} //else if(motor == SYNCHRON) {

			if (Km >= 990) {
				Km = 990;
			}

			if (timer.done()) {
				timer.stop();
				timer.start(time);

				if(motor == ASYNCHRON) {

					if (TIM3->ARR != min_ARR) {
						if (TIM3->ARR > uint16_t(4000)) {
							TIM3->ARR -= uint16_t(30);
						} else if (TIM3->ARR > uint16_t(2500)) {
							TIM3->ARR -= uint16_t(10);
						} else if (TIM3->ARR > min_ARR) {
							TIM3->ARR -= uint16_t(5);
						} else {
							TIM3->ARR++;
						}
					}

				} else if(motor == SYNCHRON) {
							if(TIM3->ARR != min_ARR) {
								if(TIM3->ARR > min_ARR) {
									if(TIM3->ARR > uint16_t(624)) {
										if(TIM3->ARR > uint16_t(1500)) {
											TIM3->ARR -= uint16_t(32);
										} else {

											TIM3->ARR -= uint16_t(3);
										}
									} else {
										TIM3->ARR-= uint16_t(1);
									}
								} else {
									TIM3->ARR++;
								}

								if(TIM3->ARR > uint16_t(624)) {
									time = 2;
								} else if (TIM3->ARR >= uint16_t(558)) {
									time = 5;
								} else if (TIM3->ARR < uint16_t(558)) {
									time = 7;
								}

							}
				} // else if(motor == SYNCHRON) {
			}
			break;
		} // switch
	} //void operator() (){

	void pusk() {

		if(motor == ASYNCHRON) {
				frequency = 3;
				Kp = 1200;
				time = 2;
				offset = 60;

		} else if(motor == SYNCHRON) {
				frequency = 5;
				Kp = 1140;
				time = 2;
				offset = 40;
		}
		Km = 5;
		TIM3->ARR = (div_f / (frequency)) - 1;

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

		HAL_TIM_Base_Start_IT(&htim3);

		timer.start(time);
		adc.measure_value();

		service.outData.error.current_S = false;
		service.outData.error.current_A = false;
		service.outData.error.current_C = false;
		service.outData.error.phase_break = false;
		service.outData.error.HV = false;

		led_red = false;
		if(motor == SYNCHRON)
			unload = true;

		reset_error = true;
	}

	void stop() {

		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

		HAL_TIM_Base_Stop_IT(&htim3);
		timer.stop();
		contactor.off();

		k = 0;
		m = 20;
		n = 40;

		state = State::wait;
		adc.measure_offset();

	}

	void alarm() {
		if((not Start or timer_stop.done()) or not contactor.is_on()
				      or service.outData.error.overheat_fc or service.outData.error.overheat_c or service.outData.error.HV_low
					  or service.outData.error.voltage_board_low or service.outData.error.voltage_board_high
		  )
		{
			if(not Start and not timer_stop.isCount()) {
//				timer_stop.start(1000);
				stop();
								timer_stop.stop();
								if (motor == SYNCHRON) {
									unload = true;
									clump_timer.start(3000);
								}
			}

			if(timer_stop.done() and not Start) {
				stop();
				timer_stop.stop();
				if (motor == SYNCHRON) {
					unload = true;
					clump_timer.start(3000);
				}
			}

			if(not contactor.is_on() or er_total
				     or service.outData.error.overheat_fc or service.outData.error.overheat_c
				     or service.outData.error.HV_low or service.outData.error.voltage_board_low or service.outData.error.voltage_board_high) {
				stop();
				timer_stop.stop();
				rerun.start(5000);
				led_red = true;
				if (motor == SYNCHRON) {
					unload = true;
					clump_timer.start(3000);
				}
			}

		}
//if(motor == SYNCHRON) {
//
//	if (TIM3->ARR >= (min_ARR + 5)) {
//		if (adc.is_error()) {
//			adc.reset_error();
//			if(error_F++ >= 2) {
//				phase = true;
//				error_F = 0;
//				error++;
//			}
//			led_red = true;
//			stop();
//			service.outData.error.phase_break = true;
//			rerun.start(5000);
//		}
//	}
//
//}

		if(adc.is_error_HV()) {
			stop();
			adc.reset_error_HV();
			led_red = true;
			service.outData.error.HV = true;
			rerun.start(5000);
			if (motor == SYNCHRON) {
				unload = true;
				clump_timer.start(3000);
			}
		}

		if(adc.is_over_s() and not service.outData.error.current_S) {
			stop();
			adc.reset_over_s();
			led_red = true;
			service.outData.error.current_S = true;
			rerun.start(5000);
			if (motor == SYNCHRON) {
				unload = true;
				clump_timer.start(3000);
			}
		}

		if(adc.is_over_a() and not service.outData.error.current_A) {
			stop();
			adc.reset_over_a();
			led_red = true;
			service.outData.error.current_A = true;
			rerun.start(5000);
			if (motor == SYNCHRON) {
				unload = true;
				clump_timer.start(3000);
			}
		}

		if(adc.is_over_c() and not service.outData.error.current_C) {
			stop();
			adc.reset_over_c();
			led_red = true;
			service.outData.error.current_C = true;
			rerun.start(5000);
			if (motor == SYNCHRON) {
				unload = true;
				clump_timer.start(3000);
			}
		}

		adc.reset_measure();
	}

};

Interrupt period_callback;
//Interrupt adc_comparator_callback;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if(htim->Instance == TIM3) //check if the interrupt comes from ACD2
	{
		period_callback.interrupt();
	}
}

//void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc){
//	if(hadc->Instance == ADC2) //check if the interrupt comes from ACD2
//	{
//		adc_comparator_callback.interrupt();
//	}
//}

