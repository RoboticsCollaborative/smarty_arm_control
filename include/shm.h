#ifndef RDDA_SHM_H
#define RDDA_SHM_H

#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>

#define SHM_SIZE    4096*2
#define AEV_NUM 6

/** AEV drive CSP Mode inputs to master */
typedef struct {
    double act_pos;
    double act_vel;
    double act_tau;
    double load_pos;
    double load_vel;
} MotorIn;

/** AEV drive CSP Mode outputs from master */
typedef struct {
    double tg_pos;
    double vel_off;
    double tau_off;
} MotorOut;

/* End-effector state */
typedef struct {
    double pos;
    double vel;
    double force;
} EE_state;

/* Arm class */
typedef struct {
    EE_state ee[AEV_NUM];
} Arm;

/** RDDAPacket transmitted using ROS */
typedef struct {
    double pos_in;
    double pos_out;
    double vel_in;
    double vel_out;
    double tau;
    double wave_in;
    double wave_out;
    double wave_in_aux;
    double wave_out_aux;
    int contact_flag;
    double test;
} RDDAPacket;

/** AEV slave class */
typedef struct {
    MotorIn motorIn;
    MotorOut motorOut;
    RDDAPacket rddaPacket;
    /* Constant */
    double init_pos;
    /* Parameter */
    double vel_sat;
    double tau_sat;
    double stiffness;
    /* SDO */
    int Pp;
    int Vp;
} AEV_slave;

/** Timestamp */
typedef struct {
    int64_t sec;
    int64_t nsec;
} Timestamp;

/** EtherCAT slave class */
typedef struct {
    AEV_slave motor[AEV_NUM];
    Arm arm[2];
    double freq_anti_alias;
    Timestamp ts;
    pthread_mutex_t mutex;
} Rdda;


Rdda *initRdda();
int mutex_lock(pthread_mutex_t *mutex);
int mutex_unlock(pthread_mutex_t *mutex);

#endif //RDDA_SHM_H