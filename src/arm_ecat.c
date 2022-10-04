/** arm_ecat.c */

#include "arm_ecat.h"

/* SOEM global vars */
char IOmap[4096];
//boolean inOP;
//boolean needlf;
//uint8 currentgroup = 0;

/** Locate and identify EtherCAT slaves.
 *
 * @param arm_slave    = Slave index group.
 * @return 0 on success.
 */
static int
slaveIdentify(ecat_slaves *slave) {
    uint16 idx = 0;
    int buf = 0;
    for (idx = 1; idx <= ec_slavecount; idx++) {
        /* AEV motor drive */
        if ((ec_slave[idx].eep_man == 0x000000ab) && (ec_slave[idx].eep_id == 0x00001260)) {
            uint32 serial_num;
            buf = sizeof(serial_num);
            ec_SDOread(idx, 0x1018, 4, FALSE, &buf, &serial_num, EC_TIMEOUTRXM);

            /* motor0 */
            // right arm base rotation motor drive
            if (serial_num == 0x0030E8D3) {
                slave->aev[0].slave_id = idx;
                /* CompleteAccess disabled for AEV drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx)) {
                    fprintf(stderr, "Motor0 mapping failed!\n");
                    exit(1);
                }
            }
            /* motor1 */
            // right arm upper arm motor drive
            else if (serial_num == 0x0030E8D4) {
                slave->aev[1].slave_id = idx;
                /* CompleteAccess disabled for AEV drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx)) {
                    fprintf(stderr, "Motor1 mapping failed!\n");
                    exit(1);
                }
            }
            /* motor2 */
            // right arm forearm motor drive
            else if (serial_num == 0x0030E8DA) {
                slave->aev[2].slave_id = idx;
                /* CompleteAccess disabled for AEV drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx)) {
                    fprintf(stderr, "Motor2 mapping failed!\n");
                    exit(1);
                }
            }

            /* motor0 */
            // left arm base rotation motor drive
            else if (serial_num == 0x01436A04) {
                slave->aev[0].slave_id = idx;
                /* CompleteAccess disabled for AEV drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx)) {
                    fprintf(stderr, "Motor3 mapping failed!\n");
                    exit(1);
                }
            }
            /* motor1 */
            // left arm upper arm motor drive
            else if (serial_num == 0x01436A01) {
                slave->aev[1].slave_id = idx;
                /* CompleteAccess disabled for AEV drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx)) {
                    fprintf(stderr, "Motor4 mapping failed!\n");
                    exit(1);
                }
            }
            /* motor2 */
            // left arm forearm motor drive
            else if (serial_num == 0x01436A03) {
                slave->aev[2].slave_id = idx;
                /* CompleteAccess disabled for AEV drive */
                //ec_slave[slaveIdx].CoEdetails ^= ECT_COEDET_SDOCA;
                /* Set PDO mapping */
                printf("Found %s at position %d\n", ec_slave[idx].name, idx);
                if (1 == mapMotorPDOs_callback(idx)) {
                    fprintf(stderr, "Motor5 mapping failed!\n");
                    exit(1);
                }
            }

            // else { printf("Serial number wrong, correct one is %d\n", serial_num);}
        }

    }

    return 0;
}

/** Initialize ETherCAT slavs memory and internal constant parameters.
 *
 * @param ecatSlave     =   EtherCAT slave structure.
 */
static void
initEcatSlaves(ecat_slaves *ecatSlave) {
    /* old motor setup */
    for (int mot_id = 0; mot_id < AEV_NUM; mot_id ++) {
        /* Input/output memory allocation */
        ecatSlave->aev[mot_id].in_motor = (motor_input *)ec_slave[ecatSlave->aev[mot_id].slave_id].inputs;
        ecatSlave->aev[mot_id].out_motor = (motor_output *)ec_slave[ecatSlave->aev[mot_id].slave_id].outputs;
        /* Constant parameters assignment */
        ecatSlave->aev[mot_id].counts_per_rad = 29213.2081;
        ecatSlave->aev[mot_id].counts_per_rad_sec = 29213.2081*10.0;
        ecatSlave->aev[mot_id].load_counts_per_rad = 3183.0989;
        ecatSlave->aev[mot_id].load_counts_per_rad_sec = 3183.0989*10;
        ecatSlave->aev[mot_id].units_per_nm = 500.0;
    }
}

/** Set up EtherCAT NIC and state machine to request all slaves to work properly.
 *
 * @param ifnameptr = NIC interface pointer
 * @return 0.
 */
ecat_slaves *initEcatConfig(void *ifnameptr) {
    char *ifname  = (char *)ifnameptr;

    int expectedWKC;
    int wkc;

    /* Initialize data structure */
    printf("Init data structure.\n");
    ecat_slaves *ecatSlaves;
    ecatSlaves = (ecat_slaves *)malloc(sizeof(ecat_slaves));
    if (ecatSlaves == NULL) {
        return NULL;
    }

    printf("Begin network configuration\n");
    /* Initialize SOEM, bind socket to ifname */
    if (ec_init(ifname)) {
	    printf("Socket connection on %s succeeded.\n", ifname);
    }
    else {
	    fprintf(stderr, "No socket connection on %s.\nExcecuted as root\n", ifname);
	    exit(1);
    }

    /* Find and configure slaves */
    if (ec_config_init(FALSE) > 0) {
	    printf("%d slaves found and configured.\n", ec_slavecount);
    }
    else {
	    fprintf(stderr, "No slaves found!\n");
	    ec_close();
	    exit(1);
    }

    /* Request for PRE-OP mode */
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    /* Locate slaves */
    slaveIdentify(ecatSlaves);

    if (ecatSlaves->aev[0].slave_id == 0 || ecatSlaves->aev[1].slave_id == 0 || ecatSlaves->aev[2].slave_id == 0) {
            fprintf(stderr, "Slaves identification failure!");
            exit(1);
    }

    /* If Complete Access (CA) disabled => auto-mapping work */
    ec_config_map(&IOmap);

    /* Let DC off for the time being */
    ec_configdc(); // DC should be launched for each identified slave

    printf("Slaves mapped, state to SAFE_OP\n");
    /* Wait for all salves to reach SAFE_OP state */
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    initMotorACD(ecatSlaves->aev[0].slave_id);
    initMotorACD(ecatSlaves->aev[1].slave_id);
    initMotorACD(ecatSlaves->aev[2].slave_id);
    printf("Slaves initialized, state to OP\n");

    /* Check if all slaves are working properly */
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);

    /* Request for OP mode */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    /* Send one valid process data to make outputs in slave happy */
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    /* el3702's second reading is not working */
    if (wkc < expectedWKC) {
        fprintf(stderr, "WKC failure.\n");
        exit(1);
    }

    /* Request OP state for all slaves */
    ec_writestate(0);

    /* Wait for all slaves to reach OP state */
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    /* Initialize slaves before use */
    initEcatSlaves(ecatSlaves);

    /* Recheck */
    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        printf("Operational state reached for all slaves\n");
        return ecatSlaves;
    }
    else {
        ec_close();
        fprintf(stderr, "Operational state failed\n");
        exit(1);
    }
}

/** Add ns to timespec.
 *
 * @param ts  =   Structure holding an interval broken down into seconds and nanoseconds
 * @param addtime   =   Elapsed time interval added to previous time.
 */
void add_timespec(struct timespec *ts, int64 addtime) {
    int64 sec, nsec;
    int64 NSEC_PER_SEC = 1000000000;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;

    if (ts->tv_nsec > NSEC_PER_SEC) {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/** PI calculation to get linux time synced to DC time
 *
 * @param reftime   =   Reference DCtime.
 * @param cycletime    =   Cycle time of PDO transfer.
 * @param offsettime    =   Offset time.
 */
int64 ec_sync(int64 reftime, int64 cycletime) {
    static int64 integral = 0;
    int64 delta;
    /* Set linux sync point 50us later than DC sync */
    assert(cycletime > 0);
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime/2)) { delta=delta-cycletime; }
    if (delta > 0) { integral++; }
    if (delta < 0) { integral--; }
    return -(delta/100)-(integral/20);
}

/** Read actual position value via SDO
 *
 * @param slave_id  =   Slave index.
 * @return position value in counts.
 */
int32 positionSDOread(uint16 slave_id) {
    int32 initial_theta1_cnts = 0;
    int size = sizeof(initial_theta1_cnts);
    ec_SDOread(slave_id, 0x2240, 0, FALSE, &size, &initial_theta1_cnts, EC_TIMEOUTRXM);
    return initial_theta1_cnts;
}

/** Read load position value via SDO
 *
 * @param slave_id  =   Slave index.
 * @return position value in counts.
 */
int32 loadPositionSDOread(uint16 slave_id) {
    int32 initial_theta1_cnts = 0;
    int size = sizeof(initial_theta1_cnts);
    ec_SDOread(slave_id, 0x2242, 0, FALSE, &size, &initial_theta1_cnts, EC_TIMEOUTRXM);
    return initial_theta1_cnts;
}

/** Write PIV gain values via SDO
 *
 * @param slave_id  =   Slave index.
 * @param Pp        =   Pp gain.
 * @param Vp        =   Vp gain.
 */
void pivGainSDOwrite(uint16 slave_id, uint16 Pp, uint16 Vp) {
    SDO_write16(slave_id, 0x2382, 1, Pp);        /* position loop gain (Pp) */
    SDO_write16(slave_id, 0x2381, 1, Vp);        /* velocity loop gain (Vp) */
}