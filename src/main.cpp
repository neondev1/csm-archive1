#include <fstream>
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

#define SENSOR_VISION 02
#define SENSOR_DIST   10
#define SENSOR_GYRO   15

#define ADI_WALL    65
#define ADI_HOOK    66
#define ADI_BALANCE 72

#define SKILLS_MATCHLOAD_T 40

using namespace pros;

Controller master(E_CONTROLLER_MASTER);
struct Drivetrain drive(
	MOTOR_LF, 
	MOTOR_LR, 
	MOTOR_RF, 
	MOTOR_RR);
Motor
	intake_l(MOTOR_INTAKE_L, 0), 
	intake_r(MOTOR_INTAKE_R, 1),
	flywheel_u(MOTOR_FLYWHEEL_U, E_MOTOR_GEAR_BLUE, 0),
	flywheel_l(MOTOR_FLYWHEEL_L, E_MOTOR_GEAR_BLUE, 0),
	flywheel_f(MOTOR_FLYWHEEL_F, E_MOTOR_GEAR_BLUE, 0);
Vision vision(SENSOR_VISION);
Distance dist(SENSOR_DIST);
Imu gyro(SENSOR_GYRO);
ADIDigitalOut
	hook(ADI_HOOK),
	balance(ADI_BALANCE),
	wall(ADI_WALL);
// Ensure that buttons are only activated once per press
int digital_l2 = 0, digital_r2 = 0, digital_a = 0, digital_x = 0, digital_left = 0;
// Intake toggle, flywheel toggle, hook toggle
int _intake = 0, _flywheel = 0, _hook = 0, _wall = 0;
// Reverse driving, turning sensitivity
int dir = 1, turnspeed = 1;
double* _cos = (double*)0;
// Team selector, autonomous picker, skills toggle
int team = 0, auton = 1, skills = 0;
char *_team = "OFF", _auton = '1', _skills = ' ';
vision_signature_s_t tri_sig1 = Vision::signature_from_utility(1, -6311, -1093, -3702, -6471, -2387, -4429, 1.000, 0);
vision_signature_s_t tri_sig2 = Vision::signature_from_utility(2, -6241, -1133, -3688, -7117, -327, -3722, 1.000, 0);

void btn2_cb(void) {
	skills = !skills;
	if (skills) {
		_team = "S";
		_auton = '0';
		_skills = 'S';
	}
	else {
		if (team == 1)
			auton = 1;
		_team = (char*)(team ? "DEF" : "OFF");
		_auton = auton ? '1' : '0';
		_skills = ' ';
	}
	lcd::print(7, "Ready | Selected program: %s%c%c", _team, _auton, _skills);
}

void initialize() {
	_cos = (double*)malloc(180 * sizeof(double));
	for (int i = 0; i < 180; i++)
		_cos[i] = cos(i * PI / 180);
	lcd::initialize();
	lcd::print(0, "Name  Port Temp | Status Monitor");
	lcd::print(1, "----------------+");
	lcd::print(7, "Initializing...");
	lcd::register_btn0_cb([] {
		team = !team;
		if (skills)
			btn2_cb();
		else {
			_team = (char*)(team ? "DEF" : "OFF");
			if (team)
				auton = 1;
			_auton = auton ? '1' : '0';
		}
		lcd::print(7, "Ready | Selected program: %s%c%c", _team, _auton, _skills);
	});
	lcd::register_btn1_cb([] {
		if (!team)
			auton = !auton;
		if (skills)
			btn2_cb();
		else
			_auton = auton ? '1' : '0';
		lcd::print(7, "Ready | Selected program: %s%c%c", _team, _auton, _skills);
	});
	lcd::register_btn2_cb(btn2_cb);
#ifndef DRIVETRAIN_2
	do {
		if (usd::is_installed()) {
			if (skills) {
				std::ifstream fs("/usd/timeout_skills.log");
				if (!fs.good())
					break;
				for (int i = 0, tmp = 0; i < (sizeof(auton_timeout) / sizeof(int))
									&& fs >> tmp; i++)
					if (tmp > auton_timeout[i])
						auton_timeout[i] = tmp;
				fs.close();
			}
			else {
				std::ifstream fs("/usd/timeout_normal.log");
				if (!fs.good())
					break;
				for (int i = 0, tmp = 0; i < (sizeof(auton_timeout) / sizeof(int))
									&& fs >> tmp; i++)
					if (tmp > auton_timeout[i])
						auton_timeout[i] = tmp;
				fs.close();
			}
		}
	} while (0);
#endif
	vision.set_signature(1, &tri_sig1);
	vision.set_signature(2, &tri_sig2);
	gyro.reset(1);
}

void disabled() {
	if (!_cos)
		initialize();
}

void competition_initialize() {
	if (!_cos)
		initialize();
}

void autonomous() {
	motor_brake_mode_e_t mode = drive.lf.get_brake_mode();
	if (!_cos)
		initialize();
	// Store delay
	double l_pos = 0,
		   r_pos = 0;
	// Initialize something
	drive.lf.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	drive.rf.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	drive.lr.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	drive.rr.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	drive.tare_position();
	gyro.tare_heading();
	unsigned now = millis();
	/*
	if (skills) {

	}
	else if (team) {

	}
	else if (auton) {
	*/
		move_mm(drive, 875, &l_pos, &r_pos);
		wait(drive, l_pos, r_pos);
		drive.lf.move_velocity(-rpm[E_MOTOR_GEAR_GREEN]);
		drive.rf.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.lr.move_velocity(-rpm[E_MOTOR_GEAR_GREEN]);
		drive.rr.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		for (;gyro.get_heading() > 285 || gyro.get_heading() < 265; delay(5));
		drive.lf.move_velocity(0);
		drive.rf.move_velocity(0);
		drive.lr.move_velocity(0);
		drive.rr.move_velocity(0);
		intake_l = -127;
		intake_r = -127;
		delay(400);
		drive.lf.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.rf.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.lr.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.rr.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		delay(1000);
		intake_l = 0;
		intake_r = 0;
		drive.lf.move_velocity(-50);
		drive.rf.move_velocity(-rpm[E_MOTOR_GEAR_GREEN]);
		drive.lr.move_velocity(-50);
		drive.rr.move_velocity(-rpm[E_MOTOR_GEAR_GREEN]);
		for (;vision.get_by_size(0).left_coord > 190
			 || vision.get_by_size(0).width < 40
			 || vision.get_object_count() < 1; delay(5));
		drive.lf.move_velocity(0);
		drive.rf.move_velocity(0);
		drive.lr.move_velocity(0);
		drive.rr.move_velocity(0);
		intake_l = 127;
		intake_r = 127;
		delay(50);
		drive.lf.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.rf.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.lr.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.rr.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		for (;vision.get_by_size(0).top_coord < 162; delay(5));
		drive.lf.move_velocity(0);
		drive.rf.move_velocity(0);
		drive.lr.move_velocity(0);
		drive.rr.move_velocity(0);
		flywheel_u = 0;
		flywheel_l = 0;
		flywheel_f = 0;
		gyro.reset(0);
		delay(750);
		intake_l = 0;
		intake_r = 0;
		drive.tare_position();
		turn_deg(drive, -70, -200, &l_pos, &r_pos);
		delay(100);
		flywheel_u = 127;
		flywheel_l = 127;
		wait(drive, l_pos, r_pos);
		flywheel_f = 127;
		intake_l = 127;
		intake_r = 127;
		delay(600);
		drive.lf.move_velocity(-rpm[E_MOTOR_GEAR_GREEN]);
		drive.rf.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.lr.move_velocity(-rpm[E_MOTOR_GEAR_GREEN]);
		drive.rr.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		delay(275);
		drive.lf.move_velocity(0);
		drive.rf.move_velocity(0);
		drive.lr.move_velocity(0);
		drive.rr.move_velocity(0);
		delay(50);
		drive.lf.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.rf.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.lr.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.rr.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		for (;vision.get_by_size(0).top_coord < 148; delay(5));
		drive.lf.move_velocity(0);
		drive.rf.move_velocity(0);
		drive.lr.move_velocity(0);
		drive.rr.move_velocity(0);
		delay(50);
		gyro.tare_heading();
		drive.lf.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.rf.move_velocity(-rpm[E_MOTOR_GEAR_GREEN]);
		drive.lr.move_velocity(rpm[E_MOTOR_GEAR_GREEN]);
		drive.rr.move_velocity(-rpm[E_MOTOR_GEAR_GREEN]);
		for (;gyro.get_heading() < 170; delay(5));
		drive.lf.move_velocity(0);
		drive.rf.move_velocity(0);
		drive.lr.move_velocity(0);
		drive.rr.move_velocity(0);
		delay(50);
		move_mm(drive, -50, &l_pos, &r_pos);
		wait(drive, l_pos, r_pos);
		hook.set_value(1);
		move_mm(drive, 200, &l_pos, &r_pos);
		wait(drive, l_pos, r_pos);
	/*
	}
	else {
	}
	*/
	intake_l = 0;
	intake_r = 0;
	flywheel_u = 0;
	flywheel_l = 0;
	flywheel_f = 0;
	drive.lf.set_brake_mode(mode);
	drive.rf.set_brake_mode(mode);
	drive.lr.set_brake_mode(mode);
	drive.rr.set_brake_mode(mode);
	lcd::print(7, "%d ms", millis() - now);
}

void opcontrol() {
	for (;;delay(10)) {
		int l_stick = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int r_stick = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		int l_motor = dir * min_max(l_stick + r_stick * dir * (turnspeed + 1) / 2, -127, 127);
		int r_motor = dir * min_max(l_stick - r_stick * dir * (turnspeed + 1) / 2, -127, 127);
		drive.lf = l_motor;
		drive.rf = r_motor;
		drive.lr = l_motor;
		drive.rr = r_motor;
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
		if (!digital_x && master.get_digital(E_CONTROLLER_DIGITAL_X)) {
			digital_x = 1;
			_wall = !_wall;
			wall.set_value(_wall);
		}
		else if (digital_x && !master.get_digital(E_CONTROLLER_DIGITAL_X))
			digital_x = 0;
		if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intake_l = 127;
			intake_r = 127;
		}
		else if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			intake_l = -127;
			intake_r = -127;
		}
		else {
			intake_l = _intake;
			intake_r = _intake;
		}
		if (master.get_digital(E_CONTROLLER_DIGITAL_UP))
			dir = 1;
		else if (master.get_digital(E_CONTROLLER_DIGITAL_DOWN))
			dir = -1;
		if (!digital_left && master.get_digital(E_CONTROLLER_DIGITAL_LEFT)) {
			digital_left = 1;
			turnspeed = !turnspeed;
		}
		else if (digital_left && !master.get_digital(E_CONTROLLER_DIGITAL_LEFT))
			digital_left = 0;
		if (master.get_digital(E_CONTROLLER_DIGITAL_B))
			balance.set_value(1);
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
			// Testing area COMMENT THIS OUT BEFORE THE COMPETITION
			// autonomous();
		}
		if (millis() % 6000 < 3000) {
			lcd::print(2, "LF     %3d%5.0f",
				MOTOR_LF * (drive.lf.get_current_draw() != PROS_ERR), drive.lf.get_temperature());
			lcd::print(3, "LR     %3d%5.0f",
				MOTOR_LR * (drive.lr.get_current_draw() != PROS_ERR), drive.lr.get_temperature());
			lcd::print(4, "RF     %3d%5.0f",
				MOTOR_RF * (drive.rf.get_current_draw() != PROS_ERR), drive.rf.get_temperature());
			lcd::print(5, "RR     %3d%5.0f",
				MOTOR_RR * (drive.rr.get_current_draw() != PROS_ERR), drive.rr.get_temperature());
			lcd::clear_line(6);
			lcd::print(7, "Ready | Selected program: %s%c%c", _team, _auton, _skills);
		}
		else {
			lcd::print(2, "INT_L  %3d%5.0f",
				MOTOR_INTAKE_L * (intake_r.get_current_draw() != PROS_ERR), intake_l.get_temperature());
			lcd::print(3, "INT_R  %3d%5.0f",
				MOTOR_INTAKE_R * (intake_r.get_current_draw() != PROS_ERR), intake_r.get_temperature());
			lcd::print(4, "FLW_U  %3d%5.0f",
				MOTOR_FLYWHEEL_U * (flywheel_u.get_current_draw() != PROS_ERR), flywheel_u.get_temperature());
			lcd::print(5, "FLW_L  %3d%5.0f",
				MOTOR_FLYWHEEL_L * (flywheel_l.get_current_draw() != PROS_ERR), flywheel_l.get_temperature());
			lcd::print(6, "FLW_F  %3d%5.0f",
				MOTOR_FLYWHEEL_F * (flywheel_f.get_current_draw() != PROS_ERR), flywheel_f.get_temperature());
			lcd::print(7,  "Ready | Selected program: %s%c%c", _team, _auton, _skills);
		}
	}
}
