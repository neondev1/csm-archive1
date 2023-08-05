#include <atomic>
#include "main.h"
#include "drivetrain.h"

#define MOTOR_LF 	    1
#define MOTOR_LR 	    2
#define MOTOR_RF 	    3
#define MOTOR_RR 	    4
#define MOTOR_INTAKE    5
#define MOTOR_FLYWHEEL1 6
#define MOTOR_FLYWHEEL2 7

#define ADI_PISTON 65

#define INTAKE_DELAY_MS /*PLACEHOLDER*/ 200

using namespace pros;

Controller master(E_CONTROLLER_MASTER);
struct Drivetrain drive(
	MOTOR_LF, 
	MOTOR_LR, 
	MOTOR_RF, 
	MOTOR_RR);
Motor intake(MOTOR_INTAKE, E_MOTOR_GEAR_BLUE);
Motor flywheel1(MOTOR_FLYWHEEL1, E_MOTOR_GEAR_BLUE);
Motor flywheel2(MOTOR_FLYWHEEL2, E_MOTOR_GEAR_BLUE);
ADIDigitalOut piston(ADI_PISTON);
int digital_l2 = 0, digital_r2 = 0, digital_a = 0;
// Intake toggle, flywheel toggle, hook toggle
int _intaket = 0, _flywheel = 0, _piston = 0;
// Intake pulse
std::atomic<int> _intakep = 0;
double* sin_tbl = (double*)0;

void initialize() {
	sin_tbl = (double*)malloc(90 * sizeof(double)); // no one cares about anything over 90 degrees
	for (int i = 0; i < 90; i++)
		sin_tbl[i] = sin(i);
}

void disabled() {
	while (1) {
		screen::print(E_TEXT_MEDIUM, 0, "This robot is currently intellectually disabled.");
		delay(500);
		screen::erase();
		delay(500);
	}
}

void competition_initialize() {
	screen::erase();
}

void autonomous() {
	
}

void opcontrol() {
	Task _ {[&] {
		while (1) {
			for (;!master.get_digital(E_CONTROLLER_DIGITAL_L1););
			_intakep = 1;
			intake = 127;
			delay(INTAKE_DELAY_MS);
			intake = _intaket;
			_intakep = 0;
			for (;master.get_digital(E_CONTROLLER_DIGITAL_L1););
		}
	}};
	int l_last = 0, r_last = 0;
	while (1) {
		int l_stick = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int r_stick = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		if (l_last != l_stick || r_last != r_stick) {
			int l_motor = min_max(l_stick + r_stick, -127, 127);
			int r_motor = min_max(l_stick - r_stick, -127, 127);
			drive.lf = l_motor;
			drive.lr = l_motor;
			drive.rf = r_motor;
			drive.rr = r_motor;
			l_last = l_stick;
			r_last = r_stick;
		}
		if (!digital_l2 && master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			digital_l2 = 1;
			_intaket = 127 * !_intaket;
		}
		else if (digital_l2 && !master.get_digital(E_CONTROLLER_DIGITAL_L2))
			digital_l2 = 0;
		if (!digital_r2 && master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			digital_r2 = 1;
			_flywheel = 127 * !_flywheel;
			flywheel1 = _flywheel;
			flywheel2 = _flywheel;
		}
		else if (digital_r2 && !master.get_digital(E_CONTROLLER_DIGITAL_R2))
			digital_r2 = 0;
		if (!digital_a && master.get_digital(E_CONTROLLER_DIGITAL_A)) {
			digital_a = 1;
			_piston = !_piston;
			piston.set_value(_piston);
		}
		else if (digital_a && !master.get_digital(E_CONTROLLER_DIGITAL_A))
			digital_a = 0;
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1))
			intake = -127;
		else if (!_intakep)
			intake = _intaket;
		delay(20);
	}
}
