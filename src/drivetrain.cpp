#include "drivetrain.h"
#include "main.h"

using namespace pros;

Drivetrain::Drivetrain(int lf, int lr, int rf, int rr) :
    lf(lf, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES), 
    lr(lr, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES), 
    rf(rf, E_MOTOR_GEAR_GREEN, 1, E_MOTOR_ENCODER_DEGREES), 
    rr(rr, E_MOTOR_GEAR_GREEN, 1, E_MOTOR_ENCODER_DEGREES) {

    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;;                                                                               ;;
    ;;         { { { { } }               { { { } }          { { }           { } }    ;;
    ;;      { { }         { }         { }        { }        { { } }       { { } }    ;;
    ;;     { { }                     { }                    { }  { }     { }  { }    ;;
    ;;    { { }                       { }                   { }   { }   { }   { }    ;;
    ;;    { }                          { { }                { }    { } { }    { }    ;;
    ;;    { }                              { } }            { }     { { }     { }    ;;
    ;;    { } }                                { }          { }      { }      { }    ;;
    ;;     { } }                                 { }        { }               { }    ;;
    ;;      { } }         { }        { }        { }         { }               { }    ;;
    ;;         { { } } } }             { { } } }            { }               } }    ;;
    ;;                                                                               ;;
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

void Drivetrain::tare_position(void) {
    lf.tare_position();
    lr.tare_position();
    rf.tare_position();
    rr.tare_position();
}

// Computes lesser of two values.
int _min(int x, int y) {
    return y ^ ((x ^ y) & -(x < y));
}

// Computes greater of two values.
int _max(int x, int y) {
    return x ^ ((x ^ y) & -(x < y));
}

// Computes absolute value.
int _abs(int num) {
    int mask = num >> 31;
    return (num ^ mask) - mask;
}
// speedy versions because why not

// Ensures that `num` is between `min` and `max`.
int min_max(int num, int min, int max) {
    return _min(_max(num, min), max);
}

// Moves an individual motor.
void move_mm(Motor& motor, double dist, int rpm) {
    screen::print(E_TEXT_MEDIUM, motor.get_port() - 16, "%f", dist * 360.0 / CIRCUMFERENCE);
    motor.move_relative(dist * 360.0 / CIRCUMFERENCE, rpm);
}

// Moves the entire drivetrain in one direction.
void move_mm(struct Drivetrain& motors, double dist) {
    move_mm(motors.lf, dist, rpm[E_MOTOR_GEAR_GREEN]);
    move_mm(motors.lr, dist, rpm[E_MOTOR_GEAR_GREEN]);
    move_mm(motors.rf, dist, rpm[E_MOTOR_GEAR_GREEN]);
    move_mm(motors.rr, dist, rpm[E_MOTOR_GEAR_GREEN]);
}

// General turning function; for fine tuning, address individual motors using `move_mm`.
void turn_deg(Drivetrain& motors, int angle, int dist, double* l_pos, double* r_pos) {
    int th = _abs(angle);
    int dir = angle > 0;
    if (!angle || (th >= 180 && dist)) // just no
        return;
    if (dist) {
        // Driving on a circular arc tangent to a ray from the centre of the robot in the direction it is facing
        double rad = sqrt(dist * dist / (2.0 - 2.0 * _cos[th]));
        double dist_out = PI * (rad + HALF_WIDTH) * (dist > 0 ? 1 : -1) * (double)th / 180.0;
        double dist_in = PI * (rad - HALF_WIDTH) * (dist > 0 ? 1 : -1) * (double)th / 180.0;
        int spd = _abs((int)((double)rpm[E_MOTOR_GEAR_GREEN] * dist_in / dist_out));
        if (l_pos)
            *l_pos = (dir ? dist_out : dist_in) * 360.0 / CIRCUMFERENCE;
        if (r_pos)
            *r_pos = (dir ? dist_in : dist_out) * 360.0 / CIRCUMFERENCE;
        move_mm(dir ? motors.lf : motors.rf, dist_out, rpm[E_MOTOR_GEAR_GREEN]);
        move_mm(dir ? motors.lr : motors.rr, dist_out, rpm[E_MOTOR_GEAR_GREEN]);
        move_mm(dir ? motors.rf : motors.lf, dist_in, spd);
        move_mm(dir ? motors.rr : motors.lr, dist_in, spd);
    }
    else {
        // Turning on the spot
        double turn = TURN_LEN * (double)angle / 360.0;
        if (l_pos)
            *l_pos = turn * 360.0 / CIRCUMFERENCE;
        if (r_pos)
            *r_pos = -turn * 360.0 / CIRCUMFERENCE;
        move_mm(motors.lf, turn, rpm[E_MOTOR_GEAR_GREEN]);
        move_mm(motors.lr, turn, rpm[E_MOTOR_GEAR_GREEN]);
        move_mm(motors.rf, -turn, rpm[E_MOTOR_GEAR_GREEN]);
        move_mm(motors.rr, -turn, rpm[E_MOTOR_GEAR_GREEN]);
    }
}

// Waits for a single motor to reach an absolute position specified by `dist`
void wait(Motor& motor, double pos) {
    for (;motor.get_position() - pos > 5.0
       || motor.get_position() - pos < -5.0;)
        delay(10);
}

// Waits until all motors have reached positions specified by `lf`, `lr`, `rf`, and `rr`
void wait(Drivetrain& motors, double l_pos, double r_pos) {
    wait(motors.lf, l_pos);
    wait(motors.lr, l_pos);
    wait(motors.rf, r_pos);
    wait(motors.rr, r_pos);
    motors.tare_position();
}
