#include "main.h"
#include "autonomous.h"

using namespace pros;

void initialize() {
	lcd::initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	Controller master(E_CONTROLLER_MASTER);
	Motors motors(1, 2, 3, 4);
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
		motors.lf = l_motor;
		motors.lr = l_motor;
		motors.rf = r_motor;
		motors.rr = r_motor;
		delay(20);
	}
}
