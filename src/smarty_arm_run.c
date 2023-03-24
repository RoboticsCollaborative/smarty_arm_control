/* arm_smartyarm.c */

#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <time.h>
#include <math.h>

#include "arm_ecat.h"
#include "arm_base.h"
#include "shm.h"
#include "smarty_arm_control.h"

volatile sig_atomic_t done = 0;
void intHandler (int sig) {
    if (sig == SIGINT) {
        done = 1;
        printf("Received interrupt");
    }
}

void arm_run (void *ifnameptr) {
    char *ifname = ifnameptr;
    /* EtherCAT struct */
    ecat_slaves *ecatSlaves;

    char* ip;
    if (*ifname == 'l') {
        ip = (char*)"enx000ec682b108";
    }
    else if (*ifname == 'r') {
        ip = (char*)"enx000ec682b14d";
    }
    else {
        printf("Type r or l for choosing smarty arm\n");
        exit(1);
    }
    
    /* Configure ethercat network and slaves. */
    ecatSlaves = initEcatConfig(ip);
    if (ecatSlaves == NULL) {
        fprintf(stderr, "Init ecatslaves failed.\n");
        exit(1);
    }
    printf("Network configuration succeed.\n");

    /* User friendly struct */
    Arm *arm;
    Wave_Prediction wp;

    /* Initialize user-friendly struct */
    arm = initArm(*ifname);
    if (arm == NULL) {
        fprintf(stderr, "Init right arm failed.\n");
        exit(1);
    }
    printf("Input/output interface succeed.\n");

    initArmStates(ecatSlaves, arm);
    arm_update(ecatSlaves, arm);
    smartyARMControlInit(&wp);

    int cycletime;

    /* timer */
    cycletime = 250; /* in microseconds */

    /* Measure time interval for sleep */
    int usec_per_sec = 1000000;
    int nsec_per_usec = 1000;
    struct timespec startTime, endTime;
    int controlInterval;
    arm_gettime(ecatSlaves);
    for (int i = 0; i < DOF/2; i ++) {
        prev_origin_shift[i] = 0.0;
    }

    while (!done) {

        /* Mark start time */
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        mutex_lock(&arm->mutex);

        arm_update(ecatSlaves, arm);
        smartyArmControl(arm, &wp, *ifname);

        /* Error code detection */
        if (!done) done = errorCheck(ecatSlaves);
        mutex_unlock(&arm->mutex);

        clock_gettime(CLOCK_MONOTONIC, &endTime);
        controlInterval = (endTime.tv_sec-startTime.tv_sec)*usec_per_sec + (endTime.tv_nsec-startTime.tv_nsec)/nsec_per_usec;
        if (controlInterval >= cycletime) {
            printf("\nControl interval time exceeds defined cycle time, CT = %d\n", controlInterval);
            continue;
        }
        arm_sleep(ecatSlaves, cycletime-controlInterval);
    }

    armStop(ecatSlaves);

}

int main(int argc, char **argv) {
    /* ctrl-c */
    signal(SIGINT, intHandler);

    pthread_t rt_thread;
    struct sched_param param;
    int policy = SCHED_FIFO;

    printf("SOEM (Simple Open EtherCAT Master)\nArm-SMARTYARM Run\n");

    if (argc > 1) {

        /* Create realtime thread */
        pthread_create(&rt_thread, NULL, (void *)&arm_run, (void *)argv[1]);
        // arm_run(argv[1]);

        /* Scheduler */
        memset(&param, 0, sizeof(param));
        param.sched_priority = 40;
        pthread_setschedparam(rt_thread, policy, &param);

        /* Core-Iso */
        cpu_set_t CPU2;
        CPU_ZERO(&CPU2);
        CPU_SET(2, &CPU2);
        pthread_setaffinity_np(rt_thread, sizeof(CPU2), &CPU2);

        /* Wait until sub-thread is finished */
        pthread_join(rt_thread, NULL);
    }
    else {
        printf("Usage: haptic_run ifname1\nifname = enp1s0 for example\n");
    }

    printf("End program\n");
    return 0;
}
