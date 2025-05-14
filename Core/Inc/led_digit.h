#pragma once

class Error_led{

	Pin& led_1;
	Pin& led_2;
	Pin& led_3;
	Pin& led_4;

	bool setting{false};

public:

	Error_led(Pin& led_1, Pin& led_2, Pin& led_3, Pin& led_4)
	: led_1{led_1}, led_2{led_2}, led_3{led_3}, led_4{led_4}
	{}

	void set(uint8_t code) {
		if(not setting) {
			setting = true;
			led_1 = code & 0b1000;
			led_2 = code & 0b0100;
			led_3 = code & 0b0010;
			led_4 = code & 0b0001;
		}
	}

	void reset(){
		led_1 = false;
		led_2 = false;
		led_3 = false;
		led_4 = false;

		setting = false;
	}

	void init(){

	}

};
