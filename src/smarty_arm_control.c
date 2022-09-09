#include "smarty_arm_control.h"

void smartyArmControl (Rdda *rdda) {

    double lu = 0.51; // upper arm length
    double elbow_h = 0.0; // elbow joint horizontal shift
    double elbow_v = 0.0; // elbow joint vertical shift
    double lf = 0.45; // forearm length
    double wrist_h = 0.28415; // wrist joint horizontal shift
    double wrist_v = 0.03648; // wrist joint vertical shift
    double upper_arm_init_offset = M_PI * 3 / 4; // motor 1
    double forearm_init_offset = M_PI / 4; // motor 2
    // double num = 3;
    double cx, cy, cz;

    Model model_init[ARM_NUM];
    double translation_pos_init[3];
    for (int i = 0; i < ARM_NUM; i ++) {
        model_init[i].c0 = 1.0;
        model_init[i].c1 = cos(upper_arm_init_offset);
        model_init[i].c2 = cos(forearm_init_offset);

        model_init[i].s0 = 0.0;
        model_init[i].s1 = sin(upper_arm_init_offset);
        model_init[i].s2 = sin(forearm_init_offset);
    }

    for (int i = 0; i < ARM_NUM; i ++) {
        translation_pos_init[0] = (-(lf * model_init[i].c2 - lu * model_init[i].c1) - elbow_h - wrist_h) * model_init[i].c0;
        translation_pos_init[1] = (-(lf * model_init[i].c2 - lu * model_init[i].c1) - elbow_h - wrist_h) * model_init[i].s0;
        translation_pos_init[2] = lu * model_init[i].s1 - lf * model_init[i].s2 + elbow_v + wrist_v;
    }
    

    Model model[ARM_NUM];
    for (int i = 0; i < ARM_NUM; i ++) {
        /* sin and cosin */
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

        /* base to endeffector rotation matrix */
        model[i].R[0][0] = model[i].c5 * (model[i].c0 * model[i].c4 - model[i].c3 * model[i].s0 * model[i].s4) - model[i].s0 * model[i].s3 * model[i].s5;
        model[i].R[0][1] = -model[i].c0 * model[i].s4 - model[i].c3 * model[i].c4 * model[i].s0;
        model[i].R[0][2] = model[i].c5 * model[i].s0 * model[i].s3 + model[i].s5 * (model[i].c0 * model[i].c4 - model[i].c3 * model[i].s0 * model[i].s4);
        model[i].R[1][0] = model[i].c5 * (model[i].c4 * model[i].s0 + model[i].c0 * model[i].c3 * model[i].s4) + model[i].c0 * model[i].s3 * model[i].s5;
        model[i].R[1][1] = model[i].c0 * model[i].c3 * model[i].c4 - model[i].s0 * model[i].s4;
        model[i].R[1][2] = model[i].s5 * (model[i].c4 * model[i].s0 + model[i].c0 * model[i].c3 * model[i].s4) - model[i].c0 * model[i].c5 * model[i].s3;
        model[i].R[2][0] = -model[i].c3 * model[i].s5 + model[i].c5 * model[i].s3 * model[i].s4;
        model[i].R[2][1] = model[i].c4 * model[i].s3;
        model[i].R[2][2] = model[i].c3 * model[i].c5 + model[i].s3 * model[i].s4 * model[i].s5;

        /* Jacobian */
        cx = 1.0 / (model[i].R[2][2] * model[i].R[2][2] + model[i].R[2][1] * model[i].R[2][1]);
        cy = 1.0 / sqrt(1 - model[i].R[2][0] * model[i].R[2][0]);
        cz = 1.0 / (model[i].R[0][0] * model[i].R[0][0] + model[i].R[1][0] * model[i].R[1][0]);

        model[i].jacobian[0][0] = -(-(lf * model[i].c2 - lu * model[i].c1) - elbow_h - wrist_h) * model[i].s0;
        model[i].jacobian[0][1] = -lu * model[i].c0 * model[i].s1;
        model[i].jacobian[0][2] = lf * model[i].c0 * model[i].s2;
        model[i].jacobian[0][3] = 0.0; model[i].jacobian[0][4] = 0.0; model[i].jacobian[0][5] = 0.0;

        model[i].jacobian[1][0] = (-(lf * model[i].c2 - lu * model[i].c1) - elbow_h - wrist_h) * model[i].c0;
        model[i].jacobian[1][1] = -lu * model[i].s0 * model[i].s1;
        model[i].jacobian[1][2] = lf * model[i].s0 * model[i].s2;
        model[i].jacobian[1][3] = 0.0; model[i].jacobian[1][4] = 0.0; model[i].jacobian[1][5] = 0.0;

        model[i].jacobian[2][0] = 0.0;
        model[i].jacobian[2][1] = lu * model[i].c1;
        model[i].jacobian[2][2] = -lf * model[i].c2;
        model[i].jacobian[2][3] = 0.0; model[i].jacobian[2][4] = 0.0; model[i].jacobian[2][5] = 0.0;

        model[i].jacobian[3][0] = 0.0; model[i].jacobian[3][1] = 0.0; model[i].jacobian[3][2] = 0.0;
        model[i].jacobian[3][3] = cx * ((model[i].c4 * model[i].c3) * model[i].R[2][2] 
                                - (-model[i].s3 * model[i].c5 + model[i].c3 * model[i].s4 * model[i].s5) * model[i].R[2][1]);
        model[i].jacobian[3][4] = cx * ((-model[i].s4 * model[i].s3) * model[i].R[2][2] - (model[i].s3 * model[i].c4 * model[i].s5) * model[i].R[2][1]);
        model[i].jacobian[3][5] = cx * (-(-model[i].c3 * model[i].s5 + model[i].s3 * model[i].s4 * model[i].c5) * model[i].R[2][1]);

        model[i].jacobian[4][0] = 0.0; model[i].jacobian[4][1] = 0.0; model[i].jacobian[4][2] = 0.0;
        model[i].jacobian[4][3] = cy * (-(model[i].c5 * model[i].c3 * model[i].s4 + model[i].s3 + model[i].s5));
        model[i].jacobian[4][4] = cy * (-(model[i].c5 * model[i].s3 * model[i].c4));
        model[i].jacobian[4][5] = cy * (-(-model[i].s5 * model[i].s3 * model[i].s4 - model[i].c3 * model[i].c5));

        model[i].jacobian[5][1] = 0.0; model[i].jacobian[5][2] = 0.0;
        model[i].jacobian[5][0] = cz * ((model[i].c5 * model[i].c4 * model[i].c0 - model[i].c5 * model[i].s0 * model[i].c3 * model[i].s4 - model[i].s0 * model[i].s3 * model[i].s5) * model[i].R[0][0] 
                                - (-model[i].c5 * model[i].s0 * model[i].c4 - model[i].c5 * model[i].c3 * model[i].c0 * model[i].s4 - model[i].c0 * model[i].s3 * model[i].s5) * model[i].R[1][0]);
        model[i].jacobian[5][3] = cz * ((-model[i].c5 * model[i].c0 * model[i].s3 * model[i]. s4 + model[i].c0 * model[i].c3 * model[i].s5) * model[i].R[0][0] 
                                - (model[i].c5 * model[i].s3 * model[i].s0 * model[i].s4 - model[i].s0 * model[i].c3 * model[i].s5) * model[i].R[1][0]);
        model[i].jacobian[5][4] = cz * ((-model[i].c5 * model[i].s4 * model[i].s0 + model[i].c0 * model[i].c3 * model[i].c4) * model[i].R[0][0] 
                                - (-model[i].c5 * model[i].c0 * model[i].s4 -model[i].c5 * model[i].c3 * model[i].s0 * model[i].c4) * model[i].R[1][0]);
        model[i].jacobian[5][5] = cz * ((-model[i].s5 * (model[i].c4 * model[i].s0 + model[i].c0 * model[i].c3 * model[i].s4) + model[i].c0 * model[i].s3 * model[i].c5) * model[i].R[0][0]
                                -(-model[i].s5 * (model[i].c0 * model[i].c4 - model[i].s3 * model[i].s0 * model[i].s4) - model[i].s0 * model[i].s3 * model[i].c5) * model[i].R[1][0]);
    }

    /* position */
    for (int i = 0; i < ARM_NUM; i ++) {
        rdda->arm[i].ee[0].pos = (-(lf * model[i].c2 - lu * model[i].c1) - elbow_h - wrist_h) * model[i].c0 - translation_pos_init[0];
        rdda->arm[i].ee[1].pos = (-(lf * model[i].c2 - lu * model[i].c1) - elbow_h - wrist_h) * model[i].s0 - translation_pos_init[1];
        rdda->arm[i].ee[2].pos = lu * model[i].s1 - lf * model[i].s2 + elbow_v + wrist_v - translation_pos_init[2];

        if (model[i].R[2][0] < 1.0 && model[i].R[2][0] > -1.0) {
            rdda->arm[i].ee[3].pos = atan2(model[i].R[2][1], model[i].R[2][2]);
            rdda->arm[i].ee[4].pos = atan2(-model[i].R[2][0], sqrt(model[i].R[2][1] * model[i].R[2][1] + model[i].R[2][2] * model[i].R[2][2]));
            rdda->arm[i].ee[5].pos = atan2(model[i].R[1][0], model[i].R[0][0]);
        }
        else if (model[i].R[2][0] == 1.0) {
            rdda->arm[i].ee[3].pos = 0.0;
            rdda->arm[i].ee[4].pos = M_PI / 2;
            rdda->arm[i].ee[5].pos = -atan2(-model[i].R[1][2], model[i].R[1][1]);
        }
        else if (model[i].R[2][0] == 1.0) {
            rdda->arm[i].ee[3].pos = 0.0;
            rdda->arm[i].ee[4].pos = -M_PI / 2;
            rdda->arm[i].ee[5].pos = atan2(-model[i].R[1][2], model[i].R[1][1]);
        }
        else {
            printf("No rotation angle solution!\n");
        }
    }

    // printf("translation x: %+lf, y: %+lf, z: %+lf, rotation x: %+lf, y: %+lf, z: %+lf\r",
        // rdda->arm[0].ee[0].pos, rdda->arm[0].ee[1].pos, rdda->arm[0].ee[2].pos, rdda->arm[0].ee[3].pos, rdda->arm[0].ee[4].pos, rdda->arm[0].ee[5].pos);


    /* velocity */
    for (int i = 0; i < ARM_NUM; i ++) {
        rdda->arm[i].ee[0].vel = model[i].jacobian[0][0] * rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_vel
                               + model[i].jacobian[0][1] * (-rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_vel)
                               + model[i].jacobian[0][2] * rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_vel;
        rdda->arm[i].ee[1].vel = model[i].jacobian[1][0] * rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_vel
                               + model[i].jacobian[1][1] * (-rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_vel)
                               + model[i].jacobian[1][2] * rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_vel;
        rdda->arm[i].ee[2].vel = model[i].jacobian[2][0] * rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_vel
                               + model[i].jacobian[2][1] * (-rdda->motor[1 + i * AEV_NUM / 2].motorIn.act_vel)
                               + model[i].jacobian[2][2] * rdda->motor[2 + i * AEV_NUM / 2].motorIn.act_vel;
        
        rdda->arm[i].ee[3].vel = model[i].jacobian[3][3] * (-rdda->motor[0 + i * AEV_NUM / 2].motorIn.load_vel)
                               + model[i].jacobian[3][4] * rdda->motor[1 + i * AEV_NUM / 2].motorIn.load_vel
                               + model[i].jacobian[3][5] * (-rdda->motor[2 + i * AEV_NUM / 2].motorIn.load_vel);
        rdda->arm[i].ee[4].vel = model[i].jacobian[4][3] * (-rdda->motor[0 + i * AEV_NUM / 2].motorIn.load_vel)
                               + model[i].jacobian[4][4] * rdda->motor[1 + i * AEV_NUM / 2].motorIn.load_vel
                               + model[i].jacobian[4][5] * (-rdda->motor[2 + i * AEV_NUM / 2].motorIn.load_vel);
        rdda->arm[i].ee[5].vel = model[i].jacobian[5][0] * rdda->motor[0 + i * AEV_NUM / 2].motorIn.act_vel
                               + model[i].jacobian[5][3] * (-rdda->motor[0 + i * AEV_NUM / 2].motorIn.load_vel)
                               + model[i].jacobian[5][4] * rdda->motor[1 + i * AEV_NUM / 2].motorIn.load_vel
                               + model[i].jacobian[5][5] * (-rdda->motor[2 + i * AEV_NUM / 2].motorIn.load_vel);

    }

    double wave_damping = 10.0;
    double ratio = 2.0;
    /* interface */
    for (int i = 0; i < ARM_NUM; i ++) {
        for (int j = 0; j < DOF / 2; j ++) {
            rdda->arm[i].ee[j].force = -1.0 * (wave_damping * rdda->arm[i].ee[j].vel - sqrt(2.0 * wave_damping) * rdda->arm[i].ptiPacket[j].wave_in) / ratio;
            rdda->arm[i].ptiPacket[j].wave_out = sqrt(2.0 * wave_damping) * rdda->arm[i].ee[j].vel - rdda->arm[i].ptiPacket[j].wave_in;
        }

        for (int j = 0; j < DOF; j ++) {
            rdda->arm[i].ptiPacket[j].pos_out = rdda->arm[i].ee[i].pos;
        }
    }

    double motor_torque_raw[3];
    /* ee force to motor torque */
    for (int i = 0; i < ARM_NUM; i ++) {
        motor_torque_raw[0] = model[i].jacobian[0][0] * rdda->arm[i].ee[0].force
                            + model[i].jacobian[1][0] * rdda->arm[i].ee[1].force
                            + model[i].jacobian[2][0] * rdda->arm[i].ee[2].force;
        motor_torque_raw[1] = -(model[i].jacobian[0][1] * rdda->arm[i].ee[0].force
                            + model[i].jacobian[1][1] * rdda->arm[i].ee[1].force
                            + model[i].jacobian[2][1] * rdda->arm[i].ee[2].force);
        motor_torque_raw[2] = model[i].jacobian[0][2] * rdda->arm[i].ee[0].force
                            + model[i].jacobian[1][2] * rdda->arm[i].ee[1].force
                            + model[i].jacobian[2][2] * rdda->arm[i].ee[2].force;
    }

    // printf("%+lf, %+lf, %+lf\r", rdda->arm[0].ee[0].force, rdda->arm[0].ee[1].force, rdda->arm[0].ee[2].force);

    for (int i = 0; i < AEV_NUM; i ++) {
        rdda->motor[i].motorOut.tau_off = saturation(MAX_TORQUE, motor_torque_raw[i]);
    }

}