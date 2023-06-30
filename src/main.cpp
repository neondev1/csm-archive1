#include "main.h"
#include "drivetrain.h"

#define MOTOR_LF 	 1
#define MOTOR_LR 	 2
#define MOTOR_RF 	 3
#define MOTOR_RR 	 4
#define MOTOR_INTAKE 5

using namespace pros;

void initialize() {
	lcd::initialize();
}

void disabled() {
	while (1) {
		lcd::set_text(1, "This robot is currently");
		lcd::set_text(2, "intellectually disabled.");
		delay(1000);
		lcd::clear_line(1);
		lcd::clear_line(2);
		delay(1000);
	}
}

void competition_initialize() {
	lcd::clear_line(1);
	lcd::clear_line(2);
	Motor intake(MOTOR_INTAKE);
	intake = 127;
}

void autonomous() {
	
}

void opcontrol() {
	Controller master(E_CONTROLLER_MASTER);
	struct Drivetrain drive(
		MOTOR_LF, 
		MOTOR_LR, 
		MOTOR_RF, 
		MOTOR_RR);
	while (true) {
		int l_stick = master.get_analog(ANALOG_LEFT_Y);
		int r_stick = master.get_analog(ANALOG_RIGHT_X);
		int l_motor = l_stick ? 
			l_stick + ((r_stick < 0) ? 0 : r_stick) :
			r_stick;
		int r_motor = l_stick ?
			l_stick + ((r_stick > 0) ? 0 : -r_stick) :
			-r_stick;
		drive.lf = l_motor;
		drive.lr = l_motor;
		drive.rf = r_motor;
		drive.rr = r_motor;
		delay(20);
	}
}
