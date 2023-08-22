#include "main.h"
#include "drivetrain.h"

#define MOTOR_LF 	     17
#define MOTOR_LR 	     18
#define MOTOR_RF 	     19
#define MOTOR_RR 	     20
#define MOTOR_INTAKE_L   01
#define MOTOR_INTAKE_R   05
#define MOTOR_FLYWHEEL_U 11
#define MOTOR_FLYWHEEL_L 13
#define MOTOR_FLYWHEEL_F 14

#define SENSOR_GYRO 10
#define SENSOR_OPTICAL 12

#define ADI_WALL    65
#define ADI_HOOK    66
#define ADI_BALANCE 72

#define AUTONOMOUS_WORKING 1
#define ODOMETER_DELAY 1 // Powers of 2 preferred

using namespace pros;

Controller master(E_CONTROLLER_MASTER);
struct Drivetrain drive(
	MOTOR_LF, 
	MOTOR_LR, 
	MOTOR_RF, 
	MOTOR_RR);
Motor
	intake_l(MOTOR_INTAKE_L, E_MOTOR_GEAR_BLUE, 0), 
	intake_r(MOTOR_INTAKE_R, E_MOTOR_GEAR_BLUE, 1),
	flywheel_u(MOTOR_FLYWHEEL_U, E_MOTOR_GEAR_BLUE, 0),
	flywheel_l(MOTOR_FLYWHEEL_L, E_MOTOR_GEAR_BLUE, 0),
	flywheel_f(MOTOR_FLYWHEEL_F, E_MOTOR_GEAR_BLUE, 0);
Imu gyro(SENSOR_GYRO);
ADIDigitalOut
	hook(ADI_HOOK),
	balance(ADI_BALANCE);
// Ensure that buttons are only activated once per press
int digital_l1 = 0, digital_l2 = 0, digital_r2 = 0, digital_a = 0, digital_x = 0;
// Intake toggle, flywheel toggle, hook toggle
int _intake = 0, _flywheel = 0, _roller, _hook = 0;
// Reverse driving, turning sensitivity
int dir = 1, turnspeed = 1;
double* _cos = (double*)0;
// Team selector, autonomous picker, skills toggle
int team = 0, auton = 1, skills = 0;
char _team = 'B', _auton = '1', _skills = ' ';
// Odometry stuffs
int16_t
	* accel_x,
	* accel_y,
    * vel_x,
    * vel_y,
	displ_x = 0,
	displ_y = 0;

void btn2_cb(void) {
	skills = !skills;
	if (skills) {
		_team = 'S';
		_auton = '0';
		_skills = 'S';
	}
	else {
		_team = team ? 'R' : 'B';
		_auton = auton ? '1' : '0';
		_skills = ' ';
	}
}

void initialize() {
	_cos = (double*)malloc(180 * sizeof(double));
	for (int i = 0; i < 180; i++)
		_cos[i] = cos(i);
	lcd::initialize();
	lcd::print(0,  "Status Monitor");
	lcd::print(1,  "Name    Port    Temp  | Name    Port    Temp");
	lcd::print(2,  "----------------------+----------------------");
	lcd::print(3,  "LF       %3d   %5.0f  | INT_L    %3d   %5.0f",
			MOTOR_LF * (drive.lf.get_current_draw() != PROS_ERR), drive.lf.get_temperature(),
			MOTOR_INTAKE_L * (intake_r.get_current_draw() != PROS_ERR), intake_l.get_temperature());
	lcd::print(4,  "LR       %3d   %5.0f  | INT_R    %3d   %5.0f",
			MOTOR_LR * (drive.lr.get_current_draw() != PROS_ERR), drive.lr.get_temperature(),
			MOTOR_INTAKE_R * (intake_r.get_current_draw() != PROS_ERR), intake_r.get_temperature());
	lcd::print(5,  "RF       %3d   %5.0f  | FLW_U    %3d   %5.0f",
			MOTOR_RF * (drive.rf.get_current_draw() != PROS_ERR), drive.rf.get_temperature(),
			MOTOR_FLYWHEEL_U * (flywheel_u.get_current_draw() != PROS_ERR), flywheel_u.get_temperature());
	lcd::print(6,  "RR       %3d   %5.0f  | FLW_L    %3d   %5.0f",
			MOTOR_RR * (drive.rr.get_current_draw() != PROS_ERR), drive.rr.get_temperature(),
			MOTOR_FLYWHEEL_L * (flywheel_l.get_current_draw() != PROS_ERR), flywheel_l.get_temperature());
	lcd::print(7,  "Initializing...       | FLW_F    %3d   %5.0f",
			MOTOR_FLYWHEEL_F * (flywheel_f.get_current_draw() != PROS_ERR), flywheel_f.get_temperature());
	lcd::register_btn0_cb([]{
		team = !team;
		if (skills)
			btn2_cb();
		else
			_team = team ? 'R' : 'B';
	});
	lcd::register_btn1_cb([]{
		auton = !auton;
		if (skills)
			btn2_cb();
		else
			_auton = auton ? '1' : '0';
	});
	lcd::register_btn2_cb(btn2_cb);
	accel_x = (int16_t*)malloc(8192 * sizeof(int16_t) / ODOMETER_DELAY);
	accel_y = (int16_t*)malloc(8192 * sizeof(int16_t) / ODOMETER_DELAY);
	vel_x = (int16_t*)malloc(8192 * sizeof(int16_t) / ODOMETER_DELAY);
	vel_x = (int16_t*)malloc(8192 * sizeof(int16_t) / ODOMETER_DELAY);
	gyro.reset(1);
	lcd::print(7,  "Ready | Selected: %c%c%c | FLW_F    %3d   %5.0f",
		_team, _auton, _skills,
		MOTOR_FLYWHEEL_F * (flywheel_f.get_current_draw() != PROS_ERR), flywheel_f.get_temperature());
}

void disabled() {
	if (!_cos)
		initialize();
}

void competition_initialize() {
	if (!_cos)
		initialize();
}

#if AUTONOMOUS_WORKING

// Distances and angles for moving
const int
	turn_0 = 90, dist_0 = 575,
	turn_1 = -45, dist_1 = 0,
	turn_2 = -90, dist_2 = 915,
	turn_3 = 0, dist_3 = 0,
	turn_4 = 0, dist_4 = 0,
	turn_5 = 0, dist_5 = 0,
	turn_6 = 0, dist_6 = 0,
	turn_7 = 0, dist_7 = 0,
	turn_8 = 0, dist_8 = 0,
	turn_9 = 0, dist_9 = 0,
	turn_a = 0, dist_a = 0;

#define TURN(n) turn_deg(drive, turn_ ## n, dist_ ## n, &l_pos, &r_pos)
#define WAIT wait(drive, l_pos, r_pos)
// yes i'm lazy i know

void accelerometer(void) {
	for (;;) {
		
		delay(ODOMETER_DELAY);
	}
}

void autonomous() {
	if (!_cos)
		initialize();
	// Store delay
	double l_pos,
		   r_pos;
	// Initialize something
	drive.tare_position();
	// Move
	TURN(0);
	// Flywheel on
	flywheel_u = 127;
	flywheel_l = 127;
	flywheel_f = 127;
	WAIT;
	// Move
	TURN(1);
	WAIT;
	TURN(2);
	WAIT;
	// Intake L, R on
	intake_l = 127;
	intake_r = 127;
	// Move
	//turn_deg(drive, turn_3, dist_3);
	// Intake L, R off
	//intake_l = 0;
	//intake_r = 0;
	// Flywheel off
	//flywheel_u = 0;
	//flywheel_l = 0;
	//flywheel_f = 0;
	// Move
	//turn_deg(drive, turn_4, dist_4);
	// Intake L on
	//intake_l = 127;
	// Move
	//turn_deg(drive, turn_4, dist_4);
	// Intake R on
	//intake_r = 127;
	// Move
	//intake lr off
	//turn+move
	//intake lr reverse
	//turn+move
	//intake lr off
	//turn only
	//move only
	//deploy hook
	//turn+move
}

#endif

void opcontrol() {
	while (1) {
		int l_stick = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int r_stick = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		int l_motor = dir * min_max(l_stick + r_stick * dir * turnspeed / 2, -127, 127);
		int r_motor = dir * min_max(l_stick - r_stick * dir * turnspeed / 2, -127, 127);
		drive.lf = l_motor;
		drive.lr = l_motor;
		drive.rf = r_motor;
		drive.rr = r_motor;
		if (!digital_l1 && master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			digital_l1 = 1;
			_roller = 127 * !_roller;
			flywheel_f = _roller;
		}
		else if (digital_l1 && !master.get_digital(E_CONTROLLER_DIGITAL_L1))
			digital_l2 = 0;
		if (!digital_l2 && master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			digital_l2 = 1;
			_intake = 127 * !_intake;
		}
		else if (digital_l2 && !master.get_digital(E_CONTROLLER_DIGITAL_L2))
			digital_l2 = 0;
		if (!digital_r2 && master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			digital_r2 = 1;
			_flywheel = 127 * !_flywheel;
			flywheel_u = _flywheel;
			flywheel_l = _flywheel;
			if (!master.get_digital(E_CONTROLLER_DIGITAL_R1))
				flywheel_f = _flywheel;
		}
		else if (digital_r2 && !master.get_digital(E_CONTROLLER_DIGITAL_R2))
			digital_r2 = 0;
		if (!digital_a && master.get_digital(E_CONTROLLER_DIGITAL_A)) {
			digital_a = 1;
			_hook = !_hook;
			hook.set_value(_hook);
		}
		else if (digital_a && !master.get_digital(E_CONTROLLER_DIGITAL_A))
			digital_a = 0;
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			intake_l = -127;
			intake_r = -127;
			flywheel_f = -127;
		}
		else {
			intake_l = _intake;
			intake_r = _intake;
		}
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP))
			dir = 1;
		else if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			dir = -1;
		if (master.get_digital(E_CONTROLLER_DIGITAL_LEFT))
			turnspeed = 1;
		else if (master.get_digital(E_CONTROLLER_DIGITAL_RIGHT))
			turnspeed = 2;
		if (master.get_digital(E_CONTROLLER_DIGITAL_B))
			balance.set_value(1);
		if (!digital_x && master.get_digital(E_CONTROLLER_DIGITAL_X)) {
			// Testing area
			autonomous();
		}
		else if (digital_x && !master.get_digital(E_CONTROLLER_DIGITAL_X))
			digital_x = 0;
		lcd::print(0,  "Status Monitor");
		lcd::print(1,  "Name    Port    Temp  | Name    Port    Temp");
		lcd::print(2,  "----------------------+----------------------");
		lcd::print(3,  "LF       %3d   %5.0f  | INT_L    %3d   %5.0f",
			MOTOR_LF * (drive.lf.get_current_draw() != PROS_ERR), drive.lf.get_temperature(),
			MOTOR_INTAKE_L * (intake_r.get_current_draw() != PROS_ERR), intake_l.get_temperature());
		lcd::print(4,  "LR       %3d   %5.0f  | INT_R    %3d   %5.0f",
			MOTOR_LR * (drive.lr.get_current_draw() != PROS_ERR), drive.lr.get_temperature(),
			MOTOR_INTAKE_R * (intake_r.get_current_draw() != PROS_ERR), intake_r.get_temperature());
		lcd::print(5,  "RF       %3d   %5.0f  | FLW_U    %3d   %5.0f",
			MOTOR_RF * (drive.rf.get_current_draw() != PROS_ERR), drive.rf.get_temperature(),
			MOTOR_FLYWHEEL_U * (flywheel_u.get_current_draw() != PROS_ERR), flywheel_u.get_temperature());
		lcd::print(6,  "RR       %3d   %5.0f  | FLW_L    %3d   %5.0f",
			MOTOR_RR * (drive.rr.get_current_draw() != PROS_ERR), drive.rr.get_temperature(),
			MOTOR_FLYWHEEL_L * (flywheel_l.get_current_draw() != PROS_ERR), flywheel_l.get_temperature());
		lcd::print(7,  "Ready | Selected: %c%c%c | FLW_F    %3d   %5.0f",
			_team, _auton, _skills,
			MOTOR_FLYWHEEL_F * (flywheel_f.get_current_draw() != PROS_ERR), flywheel_f.get_temperature());
		delay(10);
	}
}
