#ifndef ARM_BASE_H
#define ARM_BASE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "arm_ecat.h"
#include "shm.h"

void arm_update(ecat_slaves *ecatSlaves, Arm *arm);
void armStop(ecat_slaves *ecatSlaves);
int arm_gettime(ecat_slaves *ecatSlaves);
void arm_sleep(ecat_slaves *ecatSlaves, int cycletime);
void initArmStates(ecat_slaves *ecatSlaves, Arm *arm);
double saturation(double max_torque, double raw_torque);
int errorCheck(ecat_slaves *ecatSlaves);

#endif //ARM_BASE_H