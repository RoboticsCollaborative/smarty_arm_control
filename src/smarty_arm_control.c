#include "smarty_arm_control.h"
void smartyArmControl (Arm *arm, char LR) {

    double lu = 0.51; // upper arm length
    double lf = 0.45; // forearm length
    double wrist_h = 0.28415; // wrist joint horizontal shift
    double wrist_v = 0.03648; // wrist joint vertical shift
    double upper_arm_init_offset = M_PI * 157.57 / 180; // motor 1
    double forearm_init_offset = M_PI / 2; // motor 2
    double x_shift = 0.0;
    double y_shift = 0.1;
    double z_shift = 0.3;
    double pos_shift[DOF/2];
    double max_shift_vel = 0.5;
    double sample_time = 0.25e-3;

    double cx, cy, cz;

    Model model_init;
    double translation_pos_init[3];

    for (int i = 0; i < DOF/2; i ++) {
        pos_shift[i] = arm->ptiPacket[i].position_origin_shift;
        if (pos_shift[i] - prev_origin_shift[i] > 0) {
            pos_shift[i] = MIN(pos_shift[i], prev_origin_shift[i] + max_shift_vel * sample_time);
        }
        else {
            pos_shift[i] = MAX(pos_shift[i], prev_origin_shift[i] - max_shift_vel * sample_time);
        }
        prev_origin_shift[i] = pos_shift[i];
    }
    
    model_init.c0 = 1.0;
    model_init.c1 = cos(upper_arm_init_offset);
    model_init.c2 = cos(forearm_init_offset);

    model_init.s0 = 0.0;
    model_init.s1 = sin(upper_arm_init_offset);
    model_init.s2 = sin(forearm_init_offset);

    if (LR == 'r') {
        translation_pos_init[0] = -(lf * model_init.c2 - lu * model_init.c1 + wrist_h) * model_init.s0 + x_shift + pos_shift[0];
        translation_pos_init[1] = (lf * model_init.c2 - lu * model_init.c1 + wrist_h) * model_init.c0 + y_shift + pos_shift[1];
    }
    else if (LR == 'l') {
        translation_pos_init[0] = (lf * model_init.c2 - lu * model_init.c1 + wrist_h) * model_init.s0 + x_shift + pos_shift[0];
        translation_pos_init[1] = -(lf * model_init.c2 - lu * model_init.c1 + wrist_h) * model_init.c0 - y_shift + pos_shift[1];
    }
    translation_pos_init[2] = lu * model_init.s1 - lf * model_init.s2  + wrist_v + z_shift + pos_shift[2];
    

    Model model;
    
    /* sin and cosin */
    model.c0 = cos(arm->motor[0].motorIn.act_pos - arm->motor[0].init_pos);
    model.c1 = cos(-(arm->motor[1].motorIn.act_pos - arm->motor[1].init_pos) + upper_arm_init_offset);
    model.c2 = cos(arm->motor[2].motorIn.act_pos - arm->motor[2].init_pos + forearm_init_offset);
    model.c4 = cos(arm->motor[1].motorIn.load_pos - arm->motor[1].load_init_pos);
    model.c5 = cos(-(arm->motor[2].motorIn.load_pos - arm->motor[2].load_init_pos));

    model.s0 = sin(arm->motor[0].motorIn.act_pos - arm->motor[0].init_pos);
    model.s1 = sin(-(arm->motor[1].motorIn.act_pos - arm->motor[1].init_pos) + upper_arm_init_offset);
    model.s2 = sin(arm->motor[2].motorIn.act_pos - arm->motor[2].init_pos + forearm_init_offset);
    model.s4 = sin(arm->motor[1].motorIn.load_pos - arm->motor[1].load_init_pos);
    model.s5 = sin(-(arm->motor[2].motorIn.load_pos - arm->motor[2].load_init_pos));

    if (LR == 'r') {
        model.c3 = cos(arm->motor[0].motorIn.load_pos - arm->motor[0].load_init_pos);
        model.s3 = sin(arm->motor[0].motorIn.load_pos - arm->motor[0].load_init_pos);
    }
    else if (LR == 'l') {
        model.c3 = cos(-(arm->motor[0].motorIn.load_pos - arm->motor[0].load_init_pos));
        model.s3 = sin(-(arm->motor[0].motorIn.load_pos - arm->motor[0].load_init_pos));
    }

    /* base to endeffector rotation matrix */
    model.R[0][0] = model.c0 * model.c3 * model.c4 - model.s0 * model.s4;
    model.R[0][1] = -model.c5 * (model.c4 * model.s0 + model.c0 * model.c3 * model.s4) + model.c0 * model.s3 * model.s5;
    model.R[0][2] = model.s5 * (model.c4 * model.s0 + model.c0 * model.c3 * model.s4) + model.c0 * model.c5 * model.s3;
    model.R[1][0] = model.c0 * model.s4 + model.c3 * model.c4 * model.s0;
    model.R[1][1] = model.c5 * (model.c0 * model.c4 - model.c3 * model.s0 * model.s4) + model.s0 * model.s3 * model.s5;
    model.R[1][2] = model.c5 * model.s0 * model.s3 - model.s5 * (model.c0 * model.c4 - model.c3 * model.s0 * model.s4);
    model.R[2][0] = -model.c4 * model.s3;
    model.R[2][1] = model.c3 * model.s5 + model.c5 * model.s3 * model.s4;
    model.R[2][2] = model.c3 * model.c5 - model.s3 * model.s4 * model.s5;

    /* Jacobian */
    if (LR == 'r') {
        model.jacobian[0][0] = -(lf * model.c2 - lu * model.c1 + wrist_h) * model.c0;
        model.jacobian[0][1] = -lu * model.s0 * model.s1;
        model.jacobian[0][2] = lf * model.s0 * model.s2;
    }
    else if (LR == 'l') {
        model.jacobian[0][0] = (lf * model.c2 - lu * model.c1 + wrist_h) * model.c0;
        model.jacobian[0][1] = lu * model.s0 * model.s1;
        model.jacobian[0][2] = -lf * model.s0 * model.s2;
    }
    model.jacobian[0][3] = 0.0; model.jacobian[0][4] = 0.0; model.jacobian[0][5] = 0.0;

    if (LR == 'r') {
        model.jacobian[1][0] = -(lf * model.c2 - lu * model.c1 + wrist_h) * model.s0;
        model.jacobian[1][1] = lu * model.c0 * model.s1;
        model.jacobian[1][2] = -lf * model.c0 * model.s2;
    }
    else if (LR == 'l') {
        model.jacobian[1][0] = (lf * model.c2 - lu * model.c1 + wrist_h) * model.s0;
        model.jacobian[1][1] = -lu * model.c0 * model.s1;
        model.jacobian[1][2] = lf * model.c0 * model.s2;
    }
    model.jacobian[1][3] = 0.0; model.jacobian[1][4] = 0.0; model.jacobian[1][5] = 0.0;

    model.jacobian[2][0] = 0.0;
    model.jacobian[2][1] = lu * model.c1;
    model.jacobian[2][2] = -lf * model.c2;
    model.jacobian[2][3] = 0.0; model.jacobian[2][4] = 0.0; model.jacobian[2][5] = 0.0;

    // cx = 1.0 / (model.R[2][2] * model.R[2][2] + model.R[2][1] * model.R[2][1]);
    // cy = -1.0 / sqrt(1 - model.R[2][0] * model.R[2][0]);
    // cz = 1.0 / (model.R[0][0] * model.R[0][0] + model.R[1][0] * model.R[1][0]);

    // model.jacobian[3][0] = 0.0; model.jacobian[3][1] = 0.0; model.jacobian[3][2] = 0.0;
    // model.jacobian[3][3] = cx * ((-model.s3 * model.s5 + model.c5 * model.c3 * model.s4) * model.R[2][2] 
    //                         - (-model.s3 * model.c5 - model.c3 * model.s4 * model.s5) * model.R[2][1]);
    // model.jacobian[3][4] = cx * ((model.c5 * model.s3 * model.c4) * model.R[2][2]
    //                         - (-model.s3 * model.c4 * model.s5) * model.R[2][1]);
    // model.jacobian[3][5] = cx * ((model.c3 * model.c5 - model.s5 * model.s3 * model.s4) * model.R[2][2]
    //                         - (-model.c3 * model.s5 - model.s3 * model.s4 * model.c5) * model.R[2][1]);

    // model.jacobian[4][0] = 0.0; model.jacobian[4][1] = 0.0; model.jacobian[4][2] = 0.0;
    // model.jacobian[4][3] = cy * (-model.c4 * model.c3);
    // model.jacobian[4][4] = cy * (model.s4 * model.s3);
    // model.jacobian[4][5] = 0.0;

    // model.jacobian[5][1] = 0.0; model.jacobian[5][2] = 0.0; model.jacobian[5][5] = 0.0;
    // model.jacobian[5][0] = cz * ((-model.s0 * model.s4 + model.c3 * model.c3 * model.c0) * model.R[0][0] 
    //                         - (-model.s0 * model.c3 * model.c4 - model.c0 * model.s4) * model.R[1][0]);
    // model.jacobian[5][3] = cz * ((-model.s3 * model.c4 * model.s0) * model.R[0][0] 
    //                         - (-model.c0 * model.s3 * model.c4) * model.R[1][0]);
    // model.jacobian[5][4] = cz * ((model.c0 * model.c4 - model.c3 * model.s4 * model.s0) * model.R[0][0] 
    //                         - (-model.c0 * model.c3 * model.s4 - model.s0 * model.c4) * model.R[1][0]);


    cx = 1.0 / (model.R[1][2] * model.R[1][2] + model.R[1][1] * model.R[1][1]);
    cy = 1.0 / (model.R[2][0] * model.R[2][0] + model.R[0][0] * model.R[0][0]);
    cz = 1.0 / sqrt(1 - model.R[1][0] * model.R[1][0]);

    model.jacobian[3][0] = cx * (-(model.c5 * model.c0 * model.s3 - model.s5 * (-model.s0 * model.c4 - model.c3 * model.c0 * model.s4)) * model.R[1][1] 
                            + (model.c5 * (-model.s0 * model.c4 - model.c3 * model.c0 * model.s4) + model.c0 * model.s3 * model.s5) * model.R[1][2]); 
    model.jacobian[3][1] = 0.0; model.jacobian[3][2] = 0.0;
    model.jacobian[3][3] = cx * (-(model.c5 * model.s0 * model.c3 - model.s5 * model.s3 * model.s0 * model.s4) * model.R[1][1] 
                            + (model.c5 * model.s3 * model.s0 * model.s4 + model.s0 * model.c3 * model.s5) * model.R[1][2]);
    model.jacobian[3][4] = cx * (-(-model.s5 * (-model.c0 * model.s4 - model.c3 * model.s0 * model.c4)) * model.R[1][1]
                            + (model.c5 * (-model.c0 * model.s4 - model.c3 * model.s0 * model.c4)) * model.R[1][2]);
    model.jacobian[3][5] = cx * (-(-model.s5 * model.s0 * model.s3 - model.c5 * (model.c0 * model.c4 - model.c3 * model.s0 * model.s4)) * model.R[1][1]
                            + (-model.s5 * (model.c0 * model.c4 - model.c3 * model.s0 * model.s4) + model.s0 * model.s3 * model.c5) * model.R[1][2]);

    model.jacobian[4][0] = cy * ((-model.s0 * model.c3 * model.c4 - model.c0 * model.s4) * model.R[2][0]); 
    model.jacobian[4][1] = 0.0; model.jacobian[4][2] = 0.0;
    model.jacobian[4][3] = cy * (-(-model.c4 * model.c3) * model.R[0][0] + (-model.c0 * model.s3 * model.c4) * model.R[2][0]);
    model.jacobian[4][4] = cy * (-(model.s4 * model.s3) * model.R[0][0] + (-model.c0 * model.c3 * model.s4 - model.s0 * model.c4) * model.R[2][0]);
    model.jacobian[4][5] = 0.0;

    model.jacobian[5][1] = 0.0; model.jacobian[5][2] = 0.0; model.jacobian[5][5] = 0.0;
    model.jacobian[5][0] = cz * (-model.s0 * model.s4 + model.c3 * model.c4 * model.c0);
    model.jacobian[5][3] = cz * (-model.s3 * model.c4 * model.s0);
    model.jacobian[5][4] = cz * (model.c0 * model.c4 - model.c3 * model.s4 * model.s0);

    /* position */
    if (LR == 'r') {
        arm->ee[0].pos = -(lf * model.c2 - lu * model.c1 + wrist_h) * model.s0 - translation_pos_init[0];
        arm->ee[1].pos = (lf * model.c2 - lu * model.c1 + wrist_h) * model.c0 - translation_pos_init[1];
    }
    else if (LR == 'l') {
        arm->ee[0].pos = (lf * model.c2 - lu * model.c1 + wrist_h) * model.s0 - translation_pos_init[0];
        arm->ee[1].pos = -(lf * model.c2 - lu * model.c1 + wrist_h) * model.c0 - translation_pos_init[1];
    }
    arm->ee[2].pos = lu * model.s1 - lf * model.s2 + wrist_v - translation_pos_init[2];

    // arm->ee[3].pos = atan2(model.R[2][1], model.R[2][2]);
    // arm->ee[4].pos = asin(-model.R[2][0]);
    // arm->ee[5].pos = atan2(model.R[1][0], model.R[0][0]);

    arm->ee[3].pos = atan2(-model.R[1][2], model.R[1][1]);
    arm->ee[4].pos = atan2(-model.R[2][0], model.R[0][0]);
    arm->ee[5].pos = asin(model.R[1][0]);

    arm->quat.w = sqrt(1.0 + model.R[0][0] + model.R[1][1] + model.R[2][2]) / 2.0;
    arm->quat.x = (model.R[2][1] - model.R[1][2]) / (4.0 * arm->quat.w);
    arm->quat.y = (model.R[0][2] - model.R[2][0]) / (4.0 * arm->quat.w);
    arm->quat.z = (model.R[1][0] - model.R[0][1]) / (4.0 * arm->quat.w);

    /* velocity */
    arm->ee[0].vel = model.jacobian[0][0] * arm->motor[0].motorIn.act_vel
                    + model.jacobian[0][1] * (-arm->motor[1].motorIn.act_vel)
                    + model.jacobian[0][2] * arm->motor[2].motorIn.act_vel;
    arm->ee[1].vel = model.jacobian[1][0] * arm->motor[0].motorIn.act_vel
                    + model.jacobian[1][1] * (-arm->motor[1].motorIn.act_vel)
                    + model.jacobian[1][2] * arm->motor[2].motorIn.act_vel;
    arm->ee[2].vel = model.jacobian[2][0] * arm->motor[0].motorIn.act_vel
                    + model.jacobian[2][1] * (-arm->motor[1].motorIn.act_vel)
                    + model.jacobian[2][2] * arm->motor[2].motorIn.act_vel;
    

    if (LR == 'r') {
        arm->ee[3].vel = model.jacobian[3][3] * arm->motor[0].motorIn.load_vel
                        + model.jacobian[3][4] * arm->motor[1].motorIn.load_vel
                        + model.jacobian[3][5] * (-arm->motor[2].motorIn.load_vel);
        arm->ee[4].vel = model.jacobian[4][3] * arm->motor[0].motorIn.load_vel
                        + model.jacobian[4][4] * arm->motor[1].motorIn.load_vel
                        + model.jacobian[4][5] * (-arm->motor[2].motorIn.load_vel);
        arm->ee[5].vel = model.jacobian[5][0] * arm->motor[0].motorIn.act_vel
                        + model.jacobian[5][3] * arm->motor[0].motorIn.load_vel
                        + model.jacobian[5][4] * arm->motor[1].motorIn.load_vel
                        + model.jacobian[5][5] * (-arm->motor[2].motorIn.load_vel);
    }
    else if (LR == 'l') {
        arm->ee[3].vel = model.jacobian[3][3] * (-arm->motor[0].motorIn.load_vel)
                        + model.jacobian[3][4] * arm->motor[1].motorIn.load_vel
                        + model.jacobian[3][5] * (-arm->motor[2].motorIn.load_vel);
        arm->ee[4].vel = model.jacobian[4][3] * (-arm->motor[0].motorIn.load_vel)
                        + model.jacobian[4][4] * arm->motor[1].motorIn.load_vel
                        + model.jacobian[4][5] * (-arm->motor[2].motorIn.load_vel);
        arm->ee[5].vel = model.jacobian[5][0] * arm->motor[0].motorIn.act_vel
                        + model.jacobian[5][3] * (-arm->motor[0].motorIn.load_vel)
                        + model.jacobian[5][4] * arm->motor[1].motorIn.load_vel
                        + model.jacobian[5][5] * (-arm->motor[2].motorIn.load_vel);
    }


    double wave_damping = 10.0;
    double ratio[3];
    ratio[0] = 3; ratio[1] = 3; ratio[2]= 3;
    /* interface */
    for (int j = 0; j < DOF / 2; j ++) {
        arm->ee[j].force = -1.0 * (wave_damping * arm->ee[j].vel - sqrt(2.0 * wave_damping) * arm->ptiPacket[j].wave_in) / ratio[j];
        arm->ptiPacket[j].wave_out = sqrt(2.0 * wave_damping) * arm->ee[j].vel - arm->ptiPacket[j].wave_in;
    }

    for (int j = 0; j < DOF; j ++) {
        arm->ptiPacket[j].pos_out = arm->ee[j].pos;
    }

    double motor_torque_raw[3];
    /* ee force to motor torque */
    motor_torque_raw[0] = model.jacobian[0][0] * arm->ee[0].force
                        + model.jacobian[1][0] * arm->ee[1].force
                        + model.jacobian[2][0] * arm->ee[2].force;
    motor_torque_raw[1] = -(model.jacobian[0][1] * arm->ee[0].force
                        + model.jacobian[1][1] * arm->ee[1].force
                        + model.jacobian[2][1] * arm->ee[2].force);
    motor_torque_raw[2] = model.jacobian[0][2] * arm->ee[0].force
                        + model.jacobian[1][2] * arm->ee[1].force
                        + model.jacobian[2][2] * arm->ee[2].force;

    // printf("%+lf, %+lf, %+lf\r", arm->ee[0].force, arm->ee[1].force, arm->ee[2].force);

    for (int i = 0; i < AEV_NUM; i ++) {
        arm->motor[i].motorOut.tau_off = saturation(MAX_TORQUE, motor_torque_raw[i]);
    }

}