#pragma once

class Contactor {

	Pin& on_off;
	Pin& feedback;

public:

	Contactor(Pin& on_off, Pin& feedback) : on_off{on_off}, feedback{feedback} { on_off = false; }

	void on(){
		on_off = true;
	}

	void off(){
		on_off = false;
	}

	bool is_on() {
		return feedback;
	}




};
