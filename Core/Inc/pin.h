#pragma once

class Pin {
	GPIO_TypeDef* port;
	uint16_t n;

public:

	Pin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
	    port = GPIOx;
	    n = GPIO_Pin;
	  }

	  Pin(){}

	  void set()   {HAL_GPIO_WritePin(port, n, GPIO_PIN_SET);}
	  void clear() {HAL_GPIO_WritePin (port, n, GPIO_PIN_RESET);}
	  bool is_set(){return HAL_GPIO_ReadPin(port, n);}

	  void toggle() {
		  HAL_GPIO_TogglePin(port, n);
	  }

	  bool operator=(bool v)
	  {
	      v ? set() : clear();
	      return v;
	  }

	  void operator^=(bool v)
	  {
	     if (v)
	        toggle();
	  }

	  operator bool() {return is_set();}
};
