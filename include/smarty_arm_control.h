#ifndef SMARTY_ARM_CONTROL_H
#define SMARTY_ARM_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "shm.h"
#include "rdda_base.h"

#define ARM_NUM 2

typedef struct {
    double c0;
    double c1;
    double c2;
    double s0;
    double s1;
    double s2;
    double jacobian[3][3];
} Model;

void smartyArmControl(Rdda *rdda);

#endif // SMARTY_ARM_CONTROL_H