#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <signal.h>

#include "rplidar.h"

#include "common/getopt.h"
#include "common/serial.h"

#include "lcmtypes/maebot_laser_scan_t.h"

#ifndef TAKE_A_SCAN_H
#define TAKE_A_SCAN_H
#include "rplidar.h"

#define VERBOSE 1

/* Set up serial communication with the RPLIDAR and broadcast LCM on
 * an appropriate channel */
typedef struct 
{
    getopt_t *gopt;

    // Device file descriptor
    int dev;

    // Scan thread stuff
    pthread_t scan_thread;
    pthread_t stop_thread;

    // LCM stuff
    const char *channel;
    lcm_t *lcm;
} Scan_State_t;
static Scan_State_t *scan_state;

static void sig_handler(int signo)
{
    rp_lidar_stop(scan_state->dev);
}

static void* scan_loop(void *args)
{
    Scan_State_t *scan_state = (Scan_State_t*)args;

    printf("Beginning scans...\n");
    if (system("echo 1 > /sys/class/gpio/gpio122/value")) {
      printf("Error starting rplidar motor\n");
    }
    // This loops forever, barring an error
    rp_lidar_scan_timed(scan_state->dev, scan_state->lcm, scan_state->channel,3);
    printf("Terminating rplidar...\n");
    if (system("echo 0 > /sys/class/gpio/gpio122/value")) {
      printf("Error Stopping rplidar motor\n");
    }

    return NULL;
}

int take_a_scan(/*int argc, char* argv[]*/)
{
    scan_state = (Scan_State_t*) calloc(1, sizeof(Scan_State_t));

    scan_state->gopt = getopt_create();
    getopt_add_bool(scan_state->gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string(scan_state->gopt, 'd', "device", "/dev/ttyO0", "Serial device");
    getopt_add_int(scan_state->gopt, 'b', "baud", "115200", "Baud rate");
    getopt_add_string(scan_state->gopt, 'c', "channel", "MAEBOT_LASER_SCAN", "LCM channel name");

/*    if (!getopt_parse(scan_state->gopt, argc, argv, 1) || getopt_get_bool(scan_state->gopt, "help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(scan_state->gopt);
        return 0;
    }
*/
    signal(SIGTERM, sig_handler);
    signal(SIGINT, sig_handler);

    // Open device
    const char *name = getopt_get_string(scan_state->gopt, "device");
    int baud = getopt_get_int(scan_state->gopt, "baud");
    scan_state->dev = serial_open(name, baud, 1);

    if (scan_state->dev == -1) {
        printf("ERR: Could not open device at %s\n", name);
        return -1;
    }

    scan_state->lcm = lcm_create(NULL);
    scan_state->channel = getopt_get_string(scan_state->gopt, "channel");

    // Check device health
    if (rp_lidar_check_health(scan_state->dev) != HEALTH_GOOD)
        return -2;

    // Check device info
    if (VERBOSE)
        rp_lidar_check_info(scan_state->dev);

    // Begin scanning
    pthread_create(&scan_state->scan_thread, NULL, scan_loop, scan_state);
    pthread_join(scan_state->scan_thread, NULL);

    lcm_destroy(scan_state->lcm);
    getopt_destroy(scan_state->gopt);
    serial_close(scan_state->dev);
    free(scan_state);

    return 0;
}

#endif
