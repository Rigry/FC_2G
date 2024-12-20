#pragma once

#include "net_buffer.h"
#include "pin.h"

template<size_t buffer_size = 26>
class UART_
{


public:

	Pin& led_red;

	UART_(Pin& led_red) : led_red{led_red}{}

	Net_buffer<buffer_size> buffer;

	void transmit(){
//		buffer.set_size(buffer_size - DMA1_Channel3->CNDTR);
		HAL_UART_Transmit_DMA(&huart3, buffer.ptr(), buffer.size());
		led_red = true;
	}

	void receive(){
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buffer.ptr(), buffer_size);
		led_red = false;
	}

};

//(255 - DMA1_Channel3->CNDTR)

