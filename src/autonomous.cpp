#include "autonomous.h"

Motors::Motors(int lf, int lr, int rf, int rr) :
    lf(lf, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_ROTATIONS), 
    lr(lr, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_ROTATIONS), 
    rf(rf, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_ROTATIONS), 
    rr(rr, E_MOTOR_GEAR_GREEN, 0, E_MOTOR_ENCODER_ROTATIONS) {}

// Circumference of wheel is approx. 330mm
void move_mm(Motors& motors, int dist) {
    motors.lf.move_relative(((double)dist) / 330.0, 200);
    motors.lr.move_relative(((double)dist) / 330.0, 200);
    motors.rf.move_relative(((double)dist) / 330.0, 200);
    motors.rr.move_relative(((double)dist) / 330.0, 200);
}

// angle<0      - left
// angle>0      - right
// ----------------------
// direction=1  - forward
// direction=0  - none
// direction=-1 - reverse
void turn_deg(Motors& motors, double angle, int direction) {
    switch (direction) {
        case 1:
            break;
        case 0:
            break;
        case -1:
            break;
    }
}
