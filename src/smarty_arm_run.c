/* rdda_smartyarm.c */

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

#include "rdda_ecat.h"
#include "rdda_base.h"
#include "shm.h"
#include "smarty_arm_control.h"

volatile sig_atomic_t done = 0;
void intHandler (int sig) {
    if (sig == SIGINT) {
        done = 1;
        printf("Received interrupt");
    }
}

void rdda_run (void *ifnameptr) {
    char *ifname = ifnameptr;
    /* EtherCAT struct */
    ecat_slaves *ecatSlaves;
    /* User friendly struct */
    Rdda *rdda;

    int cycletime;

    /* Configure ethercat network and slaves. */
    ecatSlaves = initEcatConfig(ifname);
    if (ecatSlaves == NULL) {
        fprintf(stderr, "Init ecatslaves failed.\n");
        exit(1);
    }
    printf("Network configuration succeed.\n");

    /* Initialize user-friendly struct */
    rdda = initRdda();
    if (rdda == NULL) {
        fprintf(stderr, "Init rdda failed.\n");
        exit(1);
    }
    printf("Input/output interface succeed.\n");

    /* timer */
    cycletime = 250; /* in microseconds */

    initRddaStates(ecatSlaves, rdda);
    rdda_update(ecatSlaves, rdda);

    /* Measure time interval for sleep */
    int usec_per_sec = 1000000;
    int nsec_per_usec = 1000;
    struct timespec startTime, endTime;
    int controlInterval;
    rdda_gettime(ecatSlaves);

    while (!done) {

        /* Mark start time */
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        mutex_lock(&rdda->mutex);

        rdda_update(ecatSlaves, rdda);
        smartyArmControl(rdda);

        /* Error code detection */
        done = errorCheck(ecatSlaves);

        mutex_unlock(&rdda->mutex);

        clock_gettime(CLOCK_MONOTONIC, &endTime);
        controlInterval = (endTime.tv_sec-startTime.tv_sec)*usec_per_sec + (endTime.tv_nsec-startTime.tv_nsec)/nsec_per_usec;
        if (controlInterval >= cycletime) {
            printf("\nControl interval time exceeds defined cycle time, CT = %d\n", controlInterval);
            continue;
        }
        rdda_sleep(ecatSlaves, cycletime-controlInterval);
    }

    rddaStop(ecatSlaves);

}

int main(int argc, char **argv) {
    /* ctrl-c */
    signal(SIGINT, intHandler);

    pthread_t rt_thread;
    struct sched_param param;
    int policy = SCHED_FIFO;

    printf("SOEM (Simple Open EtherCAT Master)\nRDDA-SMARTYARM Run\n");

    if (argc > 1) {


        /* Create realtime thread */
        pthread_create(&rt_thread, NULL, (void *)&rdda_run, (void *)argv[1]);
        // rdda_run(argv[1]);

        /* Scheduler */
        memset(&param, 0, sizeof(param));
        param.sched_priority = 40;
        pthread_setschedparam(rt_thread, policy, &param);

        /* Core-Iso */
        cpu_set_t CPU3;
        CPU_ZERO(&CPU3);
        CPU_SET(3, &CPU3);
        pthread_setaffinity_np(rt_thread, sizeof(CPU3), &CPU3);

        /* Wait until sub-thread is finished */
        pthread_join(rt_thread, NULL);
    }
    else {
        printf("Usage: haptic_run ifname1\nifname = enp1s0 for example\n");
    }

    printf("End program\n");
    return 0;
}
