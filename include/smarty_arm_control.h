#ifndef SMARTY_ARM_CONTROL_H
#define SMARTY_ARM_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "shm.h"
#include "arm_base.h"

#define MAX_TORQUE 2.0

typedef struct {
    double c0;
    double c1;
    double c2;
    double c3;
    double c4;
    double c5;
    double s0;
    double s1;
    double s2;
    double s3;
    double s4;
    double s5;
    double jacobian[6][6];
    double R[3][3];
} Model;

double prev_origin_shift[DOF/2];

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

void smartyArmControl(Arm *arm, char LR);

#endif // SMARTY_ARM_CONTROL_H