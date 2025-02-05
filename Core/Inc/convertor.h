#pragma once

#include "adc.h"
#include "service.h"
#include "can.h"
#include "contactor.h"
#include "interrupt.h"
#include "pin.h"

class Convertor {
	enum State {wait, starting} state{wait};

	ADC_& adc;
	CAN<In_id, Out_id>& can;
	Service& service;
	Contactor& contactor;
	Interrupt& period_callback;
	Pin& led_red;
	Pin& ventilator;
	Pin& unload;
	Pin& TD_DM;
	Pin& Start;
	Pin& Asin_drive;
	Pin& Sin_drive;
	Pin& Conditioner;
	Pin& state_fc;
	Pin& reset_error;
	Pin& er_total;

	bool asinchron{false};
	bool sinchron{false};
	bool condition{false};

	Timer work;
	Timer rest;
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
	uint16_t frequency{3};
	uint16_t max_current{25};
	uint16_t time{2};
	uint16_t U_phase{0};
	bool U_stop{false};
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

	void period_interrupt(){

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

	}

public:

	Convertor(ADC_& adc, CAN<In_id, Out_id>& can, Service& service, Contactor& contactor, Interrupt& period_callback
			, Pin& led_red,  Pin& ventilator, Pin& unload, Pin& TD_DM,Pin& Start, Pin& Asin_drive, Pin& Sin_drive
			, Pin& Conditioner, Pin& state_fc, Pin& reset_error, Pin& er_total)
	: adc{adc}, can {can}, service{service}, contactor{contactor}, period_callback{period_callback}
	, led_red{led_red}, ventilator{ventilator}, unload{unload}, TD_DM{TD_DM}, Start{Start}, Asin_drive{Asin_drive}
	, Sin_drive{Sin_drive}, Conditioner {Conditioner}, state_fc{state_fc}, reset_error{reset_error}, er_total{er_total}
	{}

	void init() {
		rerun.time_set = 0; timer_stop.time_set = 0; clump_timer.time_set = 0;
		asinchron = bool (Asin_drive) and bool (not Sin_drive) and bool (not Conditioner);
		sinchron = bool (Sin_drive) and bool (not Asin_drive) and bool (not Conditioner);
		condition = bool (Conditioner) and bool (not Asin_drive) and bool (not Sin_drive);
		if (sinchron) {
			unload = true;
			clump_timer.start(15000);
		}
		reset_error = false;
	}

	void operator() (){

		service();

		can.outID.id_1.ARR = TIM3->ARR;
		can.outID.id_1.voltage_board = service.voltage_board;
		can.outID.id_1.voltage_phase = U_phase;
		can.outID.id_1.voltage = service.high_voltage;

		can.outID.id_2.PWM = Km/10;
		can.outID.id_2.Kp = Kp;
		can.outID.id_2.temperature = service.convertor_temp;
		can.outID.id_2.current = service.current;
		can.outID.id_2.current_drive = service.current_drive;

		can.outID.id_2.error.over_HV = (service.high_voltage > 900);
		can.outID.id_2.error.overheat_c = not bool(TD_DM);

		can.outID.id_2.state.HV_low = U_stop;
		can.outID.id_2.state.contactor = contactor.is_on();
		can.outID.id_2.state.on = Start;
		can.outID.id_2.state.voltage_board_high = (service.voltage_board > 320);
		can.outID.id_2.state.voltage_board_low = (service.voltage_board < 180);

		can.outID.id_2.state.asin_drive = asinchron;
		can.outID.id_2.state.sin_drive = sinchron;
		can.outID.id_2.state.conditioner = condition;

		can.outID.id_1.error_2.total = er_total;

		if(service.high_voltage <= 340) U_stop = true;
		else if(service.high_voltage > 350) {U_stop = false; adc.reset_error_HV();}

		if (can.outID.id_2.error.overheat_fc |= service.convertor_temp >= 75) {
			can.outID.id_2.error.overheat_fc = service.convertor_temp >= 65;
		}

		if(contactor.is_on() and enable) alarm();

		switch(state) {
		case wait:
			if(asinchron) {
				adc.set_max_current(10);
				adc.set_max_current_phase(16);
				unload = false;
				if (service.high_voltage > 340 and service.high_voltage < 520) {
					U_phase_max = ((((service.high_voltage / 20) * 990) / 141) * 115) / 100;
					min_ARR = (div_f / ((U_phase_max) * 5)) * 22; // 5/22 = 50/220
					if(min_ARR <= ARR_ASIN) min_ARR = ARR_ASIN;
				} else {
					U_phase_max = 110;
					min_ARR = ARR_ASIN;
				}
			} else if (sinchron) {
				adc.set_max_current(16);
				adc.set_max_current_phase(20);
				if(clump_timer.done()) {
					clump_timer.stop(); unload = false;
				}
				if(service.high_voltage > 340 and service.high_voltage < 520) {
					U_phase_max = ((((service.high_voltage / 20) * 940) / 141) * 115) / 100;
					min_ARR = ( (div_f / (U_phase_max)) * 50) / 70; // 70/53 = 280/212
					if(min_ARR < value_ARR) min_ARR = value_ARR;
				} else {
					U_phase_max = 212;
					min_ARR = value_ARR;
				}
			}
			enable = Start
			 and not rerun.isCount() and not rest.isCount()
			 and not can.outID.id_2.error.overheat_fc and not can.outID.id_2.error.overheat_c
			 and not can.outID.id_2.state.HV_low
			 and not can.outID.id_2.state.voltage_board_low and not can.outID.id_2.state.voltage_board_high
			 and not U_stop and not er_total
			 ;

			if(rerun.done()) rerun.stop();
			if(rest.done()) rest.stop();

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

			if(asinchron) {

				if (service.high_voltage > 340 and service.high_voltage < 520) {
					U_phase_max = ((((service.high_voltage / 20) * 990) / 141) * 115) / 100;
					min_ARR = (div_f / ((U_phase_max) * 5)) * 22; // 5/22 = 50/220
					if(min_ARR <= ARR_ASIN) min_ARR = ARR_ASIN;
				} else {
					U_phase_max = 110;
					min_ARR = ARR_ASIN;
				}

				U_phase = ((((service.high_voltage / 20) * Km) / 141) * 112) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
				Km = offset + (Kp * (div_f / TIM3->ARR) / (service.high_voltage + 1) );

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
						Kp+=3;
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
						}
					}
				}

			} else if(sinchron) {

				if (service.high_voltage > 340 and service.high_voltage < 520) {
					U_phase_max = ((((service.high_voltage / 20) * 980) / 141) * 115) / 100;
					min_ARR = ((div_f / (U_phase_max)) * 50) / 70; // 70/53 = 280/212
					if(min_ARR < value_ARR) min_ARR = value_ARR;
				} else {
					min_ARR = value_ARR;
					U_phase_max = 212;
				}

				U_phase = ((((service.high_voltage / 20) * Km) / 141) * 115) / 100; // 31 = 620 / 20; 141 = sqrt(2) * 100; 115 = добавочный
			//	U_phase += (U_phase_max - U_phase) * 10 / 50;
				Km = offset + Kp * (div_f / TIM3->ARR) / (service.high_voltage);

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

				if(asinchron) {

					if (TIM3->ARR != min_ARR) {
						if (TIM3->ARR > uint16_t(4000)) {
							TIM3->ARR -= uint16_t(40);
						} else if (TIM3->ARR > uint16_t(2500)) {
							TIM3->ARR -= uint16_t(10);
						} else if (TIM3->ARR > min_ARR) {
							TIM3->ARR -= uint16_t(5);
						} else {
							TIM3->ARR++;
						}
					}

				} else if(sinchron) {
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

		if(asinchron) {
				frequency = 3;
				Kp = 1200;
				time = 2;
				offset = 60;

		} else if(sinchron) {
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

		can.outID.id_2.error.current_S = false;
		can.outID.id_2.error.current_A = false;
		can.outID.id_2.error.current_B = false;
		can.outID.id_2.error.current_C = false;
		can.outID.id_2.error.phase_break = false;
		can.outID.id_2.error.over_HV = false;
		can.outID.id_1.error_2.limit_time = false;

		led_red = false;
		if(sinchron)
			unload = true;

		reset_error = true;

		work.start(600'000);
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
		work.stop();

		k = 0;
		m = 20;
		n = 40;

		state = State::wait;
		adc.measure_offset();
		reset_error = false;

	}

	void alarm() {
		if((not Start or timer_stop.done())
				      or er_total
				      or not contactor.is_on()
				      or can.outID.id_2.error.overheat_fc or can.outID.id_2.error.overheat_c
					  or can.outID.id_2.state.HV_low
					  or can.outID.id_2.state.voltage_board_low or can.outID.id_2.state.voltage_board_high
		) {
			if(not Start and not timer_stop.isCount()) {
				timer_stop.start(1000);
//				stop();
//				timer_stop.stop();
				if (sinchron) {
					unload = true;
					clump_timer.start(15000);
				}
			}

			if(timer_stop.done() and not Start) {
				stop();
				timer_stop.stop();
				if (sinchron) {
					unload = true;
					clump_timer.start(15000);
				}
			}

			if(not contactor.is_on()
			    or er_total
				or can.outID.id_2.error.overheat_fc or can.outID.id_2.error.overheat_c
				or can.outID.id_2.state.HV_low
				or can.outID.id_2.state.voltage_board_low or can.outID.id_2.state.voltage_board_high
			) {
				stop();
				timer_stop.stop();
				rerun.start(10000);
				led_red = true;
				if (sinchron) {
					unload = true;
					clump_timer.start(15000);
				}
			}

		}

		if (work.done() and state == starting) {
			can.outID.id_1.error_2.limit_time = true;
			stop();
			if (sinchron) {
				unload = true;
				clump_timer.start(15000);
			}
			rest.start(240'000);
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
//			rerun.start(10000);
//		}
//	}
//
//}

		if(adc.is_error_HV()) {
			stop();
			adc.reset_error_HV();
			led_red = true;
			can.outID.id_2.error.over_HV = true;
			rerun.start(10000);
			if (sinchron) {
				unload = true;
				clump_timer.start(15000);
			}
		}

		if(adc.is_over_s() and not can.outID.id_2.error.current_S) {
			stop();
			adc.reset_over_s();
			led_red = true;
			can.outID.id_2.error.current_S = true;
			rerun.start(10000);
			if (sinchron) {
				unload = true;
				clump_timer.start(15000);
			}
		}

		if(adc.is_over_a() and not can.outID.id_2.error.current_A) {
			stop();
			adc.reset_over_a();
			led_red = true;
			can.outID.id_2.error.current_A = true;
			rerun.start(10000);
			if (sinchron) {
				unload = true;
				clump_timer.start(15000);
			}
		}

		if (adc.is_over_b() and not can.outID.id_2.error.current_B) {
			stop();
			adc.reset_over_b();
			led_red = true;
			can.outID.id_2.error.current_B = true;
			rerun.start(10000);
			if (sinchron) {
				unload = true;
				clump_timer.start(15000);
			}
		}

		if(adc.is_over_c() and not can.outID.id_2.error.current_C) {
			stop();
			adc.reset_over_c();
			led_red = true;
			can.outID.id_2.error.current_C = true;
			rerun.start(10000);
			if (sinchron) {
				unload = true;
				clump_timer.start(15000);
			}
		}
		adc.reset_measure();
	}

};

Interrupt period_callback;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){
	if(htim->Instance == TIM3) //check if the interrupt comes from ACD2
	{
		period_callback.interrupt();
	}
}

