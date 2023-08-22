#pragma once
#ifndef _CSM_DRIVETRAIN_H_
#define _CSM_DRIVETRAIN_H_

#include "main.h"

#define CIRCUMFERENCE 333.0
#define TURN_LEN 956.0
#define HALF_WIDTH 155.0
#define PI 3.1416

struct Drivetrain {
    Drivetrain(int, int, int, int);
    pros::Motor lf, lr, rf, rr;
    // void offset_all(void);
    void tare_position(void);
};
const int rpm[3] = {
    100, 200, 600
};

extern double* _cos; // i am speed

int _min(int, int);
int _max(int, int);
int _abs(int);
int min_max(int, int, int);
void move_mm(pros::Motor&, double, int);
void move_mm(struct Drivetrain&, double);
void turn_deg(struct Drivetrain&, int, int, double*, double*);
void wait(pros::Motor&, double);
void wait(struct Drivetrain&, double, double);

#endif
