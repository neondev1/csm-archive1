#include "drivetrain.h"
#include "main.h"

using namespace pros;

Drivetrain::Drivetrain(int lf, int lr, int rf, int rr) :
    lf(lf, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES), 
    lr(lr, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES), 
    rf(rf, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES), 
    rr(rr, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES) {}

int _min(int x, int y) {
    return y ^ ((x ^ y) & -(x < y));
}

int _max(int x, int y) {
    return x ^ ((x ^ y) & -(x < y));
}

int min_max(int num, int min, int max) {
    return _min(_max(num, min), max);
}

void move_mm(Motor& motor, int dist, int gear) {
    motor.move_relative(360.0 * (double)dist / CIRCUMFERENCE, gear);
}

void move_mm(struct Drivetrain& motors, int dist) {
    move_mm(motors.lf, dist, rpm[E_MOTOR_GEAR_GREEN]);
    move_mm(motors.lr, dist, rpm[E_MOTOR_GEAR_GREEN]);
    move_mm(motors.rf, dist, rpm[E_MOTOR_GEAR_GREEN]);
    move_mm(motors.rr, dist, rpm[E_MOTOR_GEAR_GREEN]);
}

// angle<0 - left
// angle>0 - right
// -------------------
// sign=1  - forward
// sign=0  - neutral
// sign=-1 - reverse
void turn_deg(Drivetrain& motors, int angle, int sign, int dist) {
    if (!angle)
        move_mm(motors, dist * sign);
    else if (sign) {
        double angular = dist ? ((double)angle / 360.0) * (double)dist / sin_tbl[angle] : 0;
        double dist_out = (double)sign * (TURN_LEN_MOVING * (double)angle + angular);
        double dist_in = (double)sign * angular;
        move_mm((angle > 0) ? motors.lf : motors.rf, dist_out, rpm[E_MOTOR_GEAR_GREEN]);
        move_mm((angle > 0) ? motors.lr : motors.rr, dist_out, rpm[E_MOTOR_GEAR_GREEN]);
    }
    else {
        double turn = TURN_LEN_STILL * ((double)angle) / 360.0;
        move_mm(motors.lf, turn, rpm[E_MOTOR_GEAR_GREEN]);
        move_mm(motors.lr, turn, rpm[E_MOTOR_GEAR_GREEN]);
        move_mm(motors.rf, -turn, rpm[E_MOTOR_GEAR_GREEN]);
        move_mm(motors.rr, -turn, rpm[E_MOTOR_GEAR_GREEN]);
    }
}
