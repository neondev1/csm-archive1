#include "main.h"

using namespace pros;

void initialize() {
	lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	Controller master(E_CONTROLLER_MASTER);
	Motor lf_motor(1);
	Motor lr_motor(2);
	Motor rf_motor(3);
	Motor rr_motor(4);
	int prev = 0;
	while (true) {
		int l_stick = master.get_analog(ANALOG_LEFT_Y);
		int r_stick = master.get_analog(ANALOG_RIGHT_X);
		int l_motor = (l_stick || prev) ? 
			l_stick + ((r_stick < 0) ? 0 : r_stick) :
			r_stick;
		int r_motor = (l_stick || prev) ?
			l_stick + ((r_stick > 0) ? 0 : -r_stick) :
			-r_stick;
		prev = l_stick;
		lf_motor = l_motor;
		lr_motor = l_motor;
		rf_motor = r_motor;
		rr_motor = r_motor;
		delay(20);
	}
}
