#include "smarty_arm_control.h"

void smartyArmControl (Rdda *rdda) {
    double lu = 0.51; // upper arm length
    double lf = 0.45; // forearm length

    Model model[2];
    for (int i = 0; i < ARM_NUM; i ++) {
        model[i].c0 = cos(rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_pos);
        model[i].c1 = cos(rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_pos);
        model[i].c2 = cos(rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_pos);

        model[i].s0 = cos(rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_pos);
        model[i].s1 = cos(rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_pos);
        model[i].s2 = cos(rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_pos);

        model[i].jacobian[0][0] = -(lf * model[i].c2 - lu * model[i].c1) * model[i].s0;
        model[i].jacobian[0][1] = lu * model[i].c0 * model[i].s1;
        model[i].jacobian[0][2] = -lf * model[i].c0 * model[i].s2;
        model[i].jacobian[1][0] = (lf * model[i].c2 - lu * model[i].c1) * model[i].c0;
        model[i].jacobian[1][1] = lu * model[i].s0 * model[i].s1;
        model[i].jacobian[1][2] = -lf * model[i].s0 * model[i].s2;
        model[i].jacobian[2][0] = 0.0;
        model[i].jacobian[2][1] = lu * model[i].c1;
        model[i].jacobian[2][2] = -lf * model[i].c2;
    }

    /* position */
    for (int i = 0; i < ARM_NUM; i ++) {
        rdda->arm[i].ee[0].pos = (lf * model[i].c2 - lu * model[i].c1) * model[i].c0;
        rdda->arm[i].ee[1].pos = (lf * model[i].c2 - lu * model[i].c1) * model[i].s0;
        rdda->arm[i].ee[2].pos = lu * model[i].s1 - lf * model[i].s2;

        rdda->arm[i].ee[3].pos = rdda->motor[0 + i * AEV_NUM / 2].motorIn.load_pos;
        rdda->arm[i].ee[4].pos = rdda->motor[1 + i * AEV_NUM / 2].motorIn.load_pos;
        rdda->arm[i].ee[5].pos = rdda->motor[2 + i * AEV_NUM / 2].motorIn.load_pos;
    }

    /* velocity */
    for (int i = 0; i < ARM_NUM; i ++) {
        rdda->arm[i].ee[0].vel = model[i].jacobian[0][0] * rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_vel\
                               + model[i].jacobian[0][1] * rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_vel\
                               + model[i].jacobian[0][2] * rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_vel;
        rdda->arm[i].ee[1].vel = model[i].jacobian[1][0] * rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_vel\
                               + model[i].jacobian[1][1] * rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_vel\
                               + model[i].jacobian[1][2] * rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_vel;
        rdda->arm[i].ee[2].vel = model[i].jacobian[2][0] * rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_vel\
                               + model[i].jacobian[2][1] * rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_vel\
                               + model[i].jacobian[2][2] * rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_vel;

        rdda->arm[i].ee[3].vel = rdda->motor[0].motorIn.load_vel;
        rdda->arm[i].ee[4].vel = rdda->motor[1].motorIn.load_vel;
        rdda->arm[i].ee[5].vel = rdda->motor[2].motorIn.load_vel;
    }

    /* force */
 

}