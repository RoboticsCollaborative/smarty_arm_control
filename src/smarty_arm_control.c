#include "smarty_arm_control.h"

void smartyArmControl (Rdda *rdda) {
    double lu = 0.44; // upper arm length
    double elbow_h = 0.02947; // elbow joint horizontal shift
    double elbow_v = 0.06393; // elbow joint vertical shift
    double lf = 0.45; // forearm length
    double wrist_h = 0.28415; // wrist joint horizontal shift
    double wrist_v = 0.03648; // wrist joint vertical shift
    double upper_arm_init_offset = M_PI / 2; // motor 1
    double forearm_init_offset = 0.0; // motor 2
    // double num = 3;

    Model model[2];
    for (int i = 0; i < ARM_NUM; i ++) {
        model[i].c0 = cos(rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_pos - rdda->motor[0 + i * AEV_NUM / 2].init_pos);
        model[i].c1 = cos(-(rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_pos - rdda->motor[1 + i * AEV_NUM / 2].init_pos) + upper_arm_init_offset);
        model[i].c2 = cos(rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_pos - rdda->motor[2 + i * AEV_NUM / 2].init_pos + forearm_init_offset);
        model[i].c3 = cos(-(rdda->motor[0 + i * AEV_NUM / 2].motorIn.load_pos - rdda->motor[0 + i * AEV_NUM / 2].load_init_pos));
        model[i].c4 = cos(rdda->motor[1 + i * AEV_NUM / 2].motorIn.load_pos - rdda->motor[1 + i * AEV_NUM / 2].load_init_pos);
        model[i].c5 = cos(-(rdda->motor[2 + i * AEV_NUM / 2].motorIn.load_pos - rdda->motor[2 + i * AEV_NUM / 2].load_init_pos));

        model[i].s0 = sin(rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_pos - rdda->motor[0 + i * AEV_NUM / 2].init_pos);
        model[i].s1 = sin(-(rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_pos - rdda->motor[1 + i * AEV_NUM / 2].init_pos) + upper_arm_init_offset);
        model[i].s2 = sin(rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_pos - rdda->motor[2 + i * AEV_NUM / 2].init_pos + forearm_init_offset);
        model[i].s3 = sin(-(rdda->motor[0 + i * AEV_NUM / 2].motorIn.load_pos - rdda->motor[0 + i * AEV_NUM / 2].load_init_pos));
        model[i].s4 = sin(rdda->motor[1 + i * AEV_NUM / 2].motorIn.load_pos - rdda->motor[1 + i * AEV_NUM / 2].load_init_pos);
        model[i].s5 = sin(-(rdda->motor[2 + i * AEV_NUM / 2].motorIn.load_pos - rdda->motor[2 + i * AEV_NUM / 2].load_init_pos));

        model[i].jacobian[0][0] = -(-(lf * model[i].c2 - lu * model[i].c1) - elbow_h - wrist_h) * model[i].s0;
        model[i].jacobian[0][1] = -lu * model[i].c0 * model[i].s1;
        model[i].jacobian[0][2] = lf * model[i].c0 * model[i].s2;
        model[i].jacobian[1][0] = (-(lf * model[i].c2 - lu * model[i].c1) - elbow_h - wrist_h) * model[i].c0;
        model[i].jacobian[1][1] = -lu * model[i].s0 * model[i].s1;
        model[i].jacobian[1][2] = lf * model[i].s0 * model[i].s2;
        model[i].jacobian[2][0] = 0.0;
        model[i].jacobian[2][1] = lu * model[i].c1;
        model[i].jacobian[2][2] = -lf * model[i].c2;

        model[i].R[0][0] = model[i].c5 * (model[i].c0 * model[i].c4 - model[i].c3 * model[i].s0 * model[i].s4) - model[i].s0 * model[i].s3 * model[i].s5;
        model[i].R[0][1] = -model[i].c0 * model[i].s4 - model[i].c3 * model[i].c4 * model[i].s0;
        model[i].R[0][2] = model[i].c5 * model[i].s0 * model[i].s3 + model[i].s5 * (model[i].c0 * model[i].c4 - model[i].c3 * model[i].s0 * model[i].s4);
        model[i].R[1][0] = model[i].c5 * (model[i].c4 * model[i].s0 + model[i].c0 * model[i].c3 * model[i].s4) + model[i].c0 * model[i].s3 * model[i].s5;
        model[i].R[1][1] = model[i].c0 * model[i].c3 * model[i].c4 - model[i].s0 * model[i].s4;
        model[i].R[1][2] = model[i].s5 * (model[i].c4 * model[i].s0 + model[i].c0 * model[i].c3 * model[i].s4) - model[i].c0 * model[i].c5 * model[i].s3;
        model[i].R[2][0] = -model[i].c3 * model[i].s5 + model[i].c5 * model[i].s3 * model[i].s4;
        model[i].R[2][1] = model[i].c4 * model[i].s3;
        model[i].R[2][2] = model[i].c3 * model[i].c5 + model[i].s3 * model[i].s4 * model[i].s5;
    }

    /* position */
    for (int i = 0; i < ARM_NUM; i ++) {
        rdda->arm[i].ee[0].pos = (-(lf * model[i].c2 - lu * model[i].c1) - elbow_h - wrist_h) * model[i].c0;
        rdda->arm[i].ee[1].pos = (-(lf * model[i].c2 - lu * model[i].c1) - elbow_h - wrist_h) * model[i].s0;
        rdda->arm[i].ee[2].pos = lu * model[i].s1 - lf * model[i].s2 + elbow_v + wrist_v;

        rdda->arm[i].ee[3].pos = atan2(model[i].R[2][1], model[i].R[2][2]);
        rdda->arm[i].ee[4].pos = atan2(-model[i].R[2][0], sqrt(model[i].R[2][1] * model[i].R[2][1] + model[i].R[2][2] * model[i].R[2][2]));
        rdda->arm[i].ee[5].pos = atan2(model[i].R[1][0], model[i].R[0][0]);
    }

    printf("translation x: %+lf, y: %+lf, z: %+lf, rotation x: %+lf, y: %+lf, z: %+lf\r",
        rdda->arm[0].ee[0].pos, rdda->arm[0].ee[1].pos, rdda->arm[0].ee[2].pos, rdda->arm[0].ee[3].pos, rdda->arm[0].ee[4].pos, rdda->arm[0].ee[5].pos);


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

    }

    /* force */
 

}