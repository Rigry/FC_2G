#pragma once

#include "interrupt.h"
#include "uart.h"
#include "ntc.h"

enum Function {
      read_03  = (uint8_t)0x03
    , read_04  = (uint8_t)0x04
    , write_16 = (uint8_t)0x10
}function;

enum Error_code {
      wrong_func   = (uint8_t)0x01
    , wrong_reg    = (uint8_t)0x02
    , wrong_value  = (uint8_t)0x03
}error_code;

struct Error {
	bool current_S          : 1;
	bool current_A          : 1;
	bool current_C          : 1;
	bool HV                 : 1;
	bool on                 : 1;
	bool HV_low             : 1;
	bool overheat_c         : 1;
	bool overheat_fc        : 1;
	bool phase_break        : 1;
	bool voltage_board_low  : 1;
	bool voltage_board_high : 1;
	bool contactor          : 1;
	uint16_t res            : 4;
};

struct In_data{

};

struct Out_data{
	uint16_t voltage_board;  // 0
	uint16_t pressure;       // 1
	 int16_t PWM;            // 2
	 int16_t convertor_temp; // 3
	uint16_t current;        // 4
	uint16_t current_A;      // 5
	uint16_t current_C;      // 6
	 int16_t high_voltage;   // 7
	uint16_t max_current = 0; // 8
	uint16_t max_current_A;  // 9
	uint16_t max_current_C;  // 10
	uint16_t U_phase;         // 11
	Error error;             // 12
};

constexpr float k_adc   = 3.3 / 4095;
constexpr float k_adc_i = 3 * k_adc / 2 / 0.0167; // 3 и 2 потому что делитель 10 и 20 кОм, 0,025 В/А
constexpr float k_u     = 3.29 * 450.00 / 4095.00;

const uint16_t measure_time{25};
constexpr uint16_t qty_measure = 200 / measure_time;

template<class In_data_t, class Out_data_t>
class Service
{
	ADC_& adc;
	NTC& ntc;
	UART_<>& uart;
	Interrupt& interrupt_DMA;
	Interrupt& interrupt_usart;

	uint16_t arr_new_hv[qty_measure] { 0 };
	uint8_t m { 0 };
	int16_t HV_avarage { 0 };

	Timer timer;
	Timer measure_timer { measure_time };

	float new_hv{0};
	bool event{false};
	bool kolhoz{false};

	void uartInterrupt(){
		event = true;
		timer.stop();
	}

	void dmaInterrupt(){
		uart.receive();
	}

	using Parent = Service;

	struct uart_interrupt: Interrupting {
		Parent &parent;
		uart_interrupt(Parent &parent) :
				parent(parent) {
			parent.interrupt_usart.subscribe(this);
		}
		void interrupt() override {
			parent.uartInterrupt();
		}
	} uart_ { *this };

	struct dma_interrupt: Interrupting {
		Parent &parent;
		dma_interrupt(Parent &parent) :
				parent(parent) {
			parent.interrupt_DMA.subscribe(this);
		}
		void interrupt() override {
			parent.dmaInterrupt();
		}
	} dma_ {*this};

public:

	static constexpr uint16_t InDataQty = sizeof(In_data_t) / 2;
	static constexpr uint16_t OutDataQty = sizeof(Out_data_t) / 2;

	union {
		In_data_t inData;
		uint16_t arInData[InDataQty];
	};
	union {
		Out_data_t outData;
		uint16_t arOutData[OutDataQty];
	};
	union {
		In_data_t inDataMin;
		uint16_t arInDataMin[InDataQty];
	};
	union {
		In_data_t inDataMax;
		uint16_t arInDataMax[InDataQty];
	};

	Service (
		  ADC_& adc
		, NTC& ntc
		, UART_<>& uart
		, Interrupt& interrupt_DMA
		, Interrupt& interrupt_usart
	) : adc              {adc}
	  , ntc              {ntc}
	  ,	uart             {uart}
	  , interrupt_DMA    {interrupt_DMA}
      , interrupt_usart  {interrupt_usart}
      , arInData { }, arOutData { }, arInDataMin { }, arInDataMax {}
	{
		uart.receive();
		timer.start(500);
	}

	void operator()(){

		outData.voltage_board  = k_adc * adc[V24] * 100 + 3; // 3 падение на диоде
		outData.convertor_temp  = ntc(adc[Trad]);

		outData.current        = (abs(adc.value(PS) - adc.offset_I_S)) * 100 / 21;
		outData.current_A      = adc.current();

//		new_hv = static_cast<float>(adc.value_HV()) * k_u;
//		outData.high_voltage  += (new_hv - outData.high_voltage) * 1 / 3;

		new_hv = (adc.value_HV() * 350 / 4095 * 45) / 10;
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
			outData.high_voltage += (HV_avarage - outData.high_voltage) * 10 / 40;
		}

		kolhoz ^= timer.event();

		if (event or kolhoz) {
			if(uart.buffer[0] == 4 or kolhoz) {
				uart.buffer.clear();
				uart.buffer << outData.voltage_board
						    << outData.pressure
							<< outData.PWM
							<< outData.convertor_temp
							<< outData.current
							<< outData.current_A
							<< outData.current_C
							<< outData.high_voltage
							<< outData.max_current
							<< outData.max_current_A
							<< outData.max_current_C
							<< outData.U_phase
							<< arOutData[12];

			} else if(uart.buffer[0] == '+') {
				uart.buffer.clear();
				uart.buffer << 'O';
				uart.buffer << 'K';
			}
			event = false;
			kolhoz = false;
			if(uart.buffer.size())
				uart.transmit();
			else
				uart.receive();
		}
	}

};

Interrupt interrupt_dma;
Interrupt interrupt_uart;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		interrupt_dma.interrupt();
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART3) {
		interrupt_uart.interrupt();
	}
}
