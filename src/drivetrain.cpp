#include "drivetrain.h"

Drivetrain::Drivetrain(int lf, int lr, int rf, int rr) :
    lf(lf, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES), 
    lr(lr, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES), 
    rf(rf, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES), 
    rr(rr, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_DEGREES) {}

void move_mm(Motor motor, int dist, int gear) {
    motor.move_relative(360.0 * ((double)dist) / CIRCUMFERENCE, gear);
}

void move_mm(struct Drivetrain& motors, int dist) {
    move_mm(motors.lf, dist, 200);
    move_mm(motors.lr, dist, 200);
    move_mm(motors.rf, dist, 200);
    move_mm(motors.rr, dist, 200);
}

// angle<0      - left
// angle>0      - right
// -------------------------
// direction=1  - forward
// direction=0  - none
// direction=-1 - reverse
void turn_deg(Drivetrain& motors, double angle, int direction) {
    switch (direction) {
        case 1:
            break;
        case 0:
            double dist = TURN_LEN_STILL * angle;
            move_mm(motors.lf, dist, 200);
            move_mm(motors.lr, dist, 200);
            move_mm(motors.rf, dist, 200);
            move_mm(motors.rr, dist, 200);
            break;
        case -1:
            break;
    }
}
