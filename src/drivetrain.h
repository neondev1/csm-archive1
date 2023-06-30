#include "main.h"
using namespace pros;

#ifndef _CSM_DRIVETRAIN_H_
#define _CSM_DRIVETRAIN_H_

#define CIRCUMFERENCE 330.0
#define TURN_LEN_STILL /*PLACEHOLDER*/ 262556.2
#define TURN_LEN_MOVING /*PLACEHOLDER*/ 262556.2

struct Drivetrain {
    Drivetrain(int, int, int, int);
    Motor lf, lr, rf, rr;
};

void move_mm(Motor, int, int);
void move_mm(struct Drivetrain&, int);
void turn_deg(struct Drivetrain&, double, int);

#endif
