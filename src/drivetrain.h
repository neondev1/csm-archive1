#pragma once
#ifndef _CSM_DRIVETRAIN_H_
#define _CSM_DRIVETRAIN_H_

#include "main.h"

#define CIRCUMFERENCE 330.0
#define TURN_LEN_STILL /*PLACEHOLDER*/ 0x28101A
#define TURN_LEN_MOVING /*PLACEHOLDER*/ 0x28101A

struct Drivetrain {
    Drivetrain(int, int, int, int);
    pros::Motor lf, lr, rf, rr;
};

const int rpm[3] = {
    100, 200, 600
};
extern double* sin_tbl; // i am speed

int _min(int, int);
int _max(int, int);
int min_max(int, int, int);
void move_mm(pros::Motor&, int, int);
void move_mm(struct Drivetrain&, int);
void turn_deg(struct Drivetrain&, int, int, int);

#endif
