#include "drivetrain.h"
#include "main.h"

#ifdef DRIVETRAIN_2

using namespace pros;

// Moves the entire drivetrain in one direction.
void move_mm(struct Drivetrain& motors, double dist, double* l_pos, double* r_pos) {
    motors.lf.move_velocity(dist);
    motors.rf.move_velocity(dist);
    motors.lr.move_velocity(dist);
    motors.rr.move_velocity(dist);
    double pos = dist * 360.0 / CIRCUMFERENCE;
    if (l_pos)
        *l_pos = pos;
    if (r_pos)
        *r_pos = pos;
}

// General turning function; for fine tuning, address individual motors using `move_mm`.
void turn_deg(Drivetrain& motors, int angle, int dist, double* l_pos, double* r_pos) {
    int th = _abs(angle);
    int dir = angle > 0;
    if (!angle || (th >= 180 && dist)) // just no
        return;
    if (dist) {
        // Driving on a circular arc tangent to a ray from the centre of the robot in the direction it is facing
        int sign = dist < 0;
        double rad = sqrt(dist * dist / (2.0 - 2.0 * _cos[th]));
        double dist_out = PI * (rad + HALF_WIDTH) * (dist > 0 ? 1 : -1) * (double)th / 180.0;
        double dist_in = PI * (rad - HALF_WIDTH) * (dist > 0 ? 1 : -1) * (double)th / 180.0;
        int in_spd = (int)((double)rpm[E_MOTOR_GEAR_GREEN] * dist_in / dist_out);
        if (sign)
            dir = !dir;
        if (l_pos)
            *l_pos = (dir ? dist_out : dist_in) * 360.0 / CIRCUMFERENCE;
        if (r_pos)
            *r_pos = (dir ? dist_in : dist_out) * 360.0 / CIRCUMFERENCE;
        int left_spd = (1 - sign * 2) * (dir ? rpm[E_MOTOR_GEAR_GREEN] : in_spd);
        int right_spd = (1 - sign * 2) * (dir ? in_spd : rpm[E_MOTOR_GEAR_GREEN]);
        motors.lf.move_velocity(left_spd);
        motors.rf.move_velocity(right_spd);
        motors.lr.move_velocity(left_spd);
        motors.rr.move_velocity(right_spd);
    }
    else {
        // Turning on the spot
        double turn = TURN_LEN * (double)angle / 360.0;
        if (l_pos)
            *l_pos = turn * 360.0 / CIRCUMFERENCE;
        if (r_pos)
            *r_pos = -turn * 360.0 / CIRCUMFERENCE;
        int spd = (dir * 2 - 1) * rpm[E_MOTOR_GEAR_GREEN];
        motors.lf.move_velocity(spd);
        motors.rf.move_velocity(-spd);
        motors.lr.move_velocity(spd);
        motors.rr.move_velocity(-spd);
    }
}

// Waits for a single motor to reach an absolute position specified by `dist`
void wait(Motor& motor, double pos) {
    if (pos > 0)
        for (;motor.get_position() < pos - 5.0; delay(5))
            lcd::print(0, "%09.0f/%09.0f", motor.get_position(), pos);
    else
        for (;motor.get_position() > pos + 5.0; delay(5))
            lcd::print(0, "%09.0f/%09.0f", motor.get_position(), pos);
    motor.move_velocity(0);
}

// Waits until all motors have reached positions specified by `lf`, `lr`, `rf`, and `rr`
void wait(Drivetrain& motors, double l_pos, double r_pos) {
    wait(motors.lf, l_pos);
    wait(motors.rf, r_pos);
    wait(motors.lr, l_pos);
    wait(motors.rr, r_pos);
    motors.tare_position();
}

#endif
