#include "main.h"
using namespace pros;

#ifndef __CSM_AUTONOMOUS_H__
#define __CSM_AUTONOMOUS_H__

class Motors {
public:
    Motors(int, int, int, int);
    Motor lf, lr, rf, rr;
};

void move_mm(Motors&, int);
void turn_deg(Motors&, double, int);

#endif
