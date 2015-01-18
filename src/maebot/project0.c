#include <lcm/lcm.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <inttypes.h>
#include <pthread.h>
#include <unistd.h>

#include "common/timestamp.h"
#include "common/getopt.h"
#include "common/serial.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "rplidar.h"
#include "take_a_pic.h"

#define PERIOD 50000 //us  -> 20Hz
#define MOTOR_SPEED 0.25f
#define MOTOR_STOP 0.0f

#define WHEEL_RADIUS 0.032f
#define WHEELBASE 0.08f
#define TICKS_REV 480
#define TICKS_FOOT 1456 

#define VERBOSE 1

// thread to drive motors
maebot_motor_command_t motor_msg;
pthread_mutex_t motor_msg_mutex;

//keep track of ticks and pics
typedef struct{
	int32_t left;
	int32_t right;
	int pic;
}State;

State state;

typedef struct Rp_Run_State
{
    getopt_t *gopt;

    // Device file descriptor
    int dev;

    // Scan thread stuff
    pthread_t scan_thread;

    // LCM stuff
    const char *channel;
    lcm_t *lcm;
} rp_state_t;
static rp_state_t *rp_run_state;



void *
diff_drive_thread (void *arg)
{
	lcm_t *lcm = lcm_create (NULL);

    uint64_t utime_start;
    while(1) {
        utime_start = utime_now (); 

        pthread_mutex_lock (&motor_msg_mutex);
        maebot_motor_command_t_publish (lcm, "MAEBOT_MOTOR_COMMAND", &motor_msg);
        pthread_mutex_unlock (&motor_msg_mutex);

		usleep (PERIOD - (utime_now() - utime_start));
    }   

    return NULL;
}

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const maebot_motor_feedback_t *msg, void *user)
{
    // int res = system ("clear");
    // if (res)
    //     printf ("system clear failed\n");

    // printf ("Subscribed to channel: %s\n", channel);
    // printf ("utime: %"PRId64"\n", msg->utime);
    // printf ("encoder_[left, right]_ticks:\t\t%d,\t%d\n",
    //         msg->encoder_left_ticks, msg->encoder_right_ticks);
    // printf ("motor_current[left, right]:\t\t%d,\t%d\n",
    //         msg->motor_current_left, msg->motor_current_right);
    // printf ("motor_[left, right]_commanded_speed:\t%f,\t%f\n",
    //         msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
    // printf ("motor_[left, right]_actual_speed:\t%f,\t%f\n",
    //         msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);

	state.left = msg->encoder_left_ticks;
	state.right = msg->encoder_right_ticks;
}



void drive_forward(int dist){

	lcm_t *lcm = lcm_create (NULL);

	maebot_motor_feedback_t_subscribe (lcm,
                 	              	"MAEBOT_MOTOR_FEEDBACK",
								   	motor_feedback_handler,
									NULL);

	lcm_handle (lcm);

	//move
	pthread_mutex_lock (&motor_msg_mutex);
    motor_msg.motor_left_speed  = MOTOR_SPEED;
	motor_msg.motor_right_speed = MOTOR_SPEED; 
	pthread_mutex_unlock (&motor_msg_mutex);

	usleep(200000); 
	lcm_handle (lcm);
	//store value of ticks before moving
	int32_t left_init = state.left;
	int32_t right_init = state.right;

	int32_t left_change;
	int32_t right_change;
	int32_t diff;
	
	//keep updating state while not met distance target
	int32_t target = left_init + (dist * TICKS_FOOT);
	while(state.left < target){

		lcm_handle (lcm);
		
		left_change = state.left - left_init;
		right_change = state.right - right_init;
		diff = left_change - right_change;
		
		if(diff < -150)		//seems to have probs otherwise
				diff = -150;

		pthread_mutex_lock (&motor_msg_mutex);
    		motor_msg.motor_left_speed  = MOTOR_SPEED;
		motor_msg.motor_right_speed = MOTOR_SPEED * (1.0f + (float)diff/800.0f); 
		pthread_mutex_unlock (&motor_msg_mutex);
			
	}
	
	//stop
	pthread_mutex_lock (&motor_msg_mutex);
    	motor_msg.motor_left_speed  = MOTOR_STOP;
	motor_msg.motor_right_speed = MOTOR_STOP; 
	pthread_mutex_unlock (&motor_msg_mutex);

	lcm_handle(lcm);
}

void turn_left(){
	lcm_t *lcm = lcm_create (NULL);

	maebot_motor_feedback_t_subscribe (lcm,
                 	"MAEBOT_MOTOR_FEEDBACK",
			motor_feedback_handler,
			NULL);


	lcm_handle(lcm);
	int32_t wheels_diff, desired_diff;
	desired_diff = 450;
	wheels_diff = state.right - state.left;

	//turn left
	pthread_mutex_lock (&motor_msg_mutex);
    motor_msg.motor_left_speed  = -MOTOR_SPEED * 0.75f;
	motor_msg.motor_right_speed = MOTOR_SPEED * 0.75f; 
	pthread_mutex_unlock (&motor_msg_mutex);
	
	while (1) {
		lcm_handle(lcm);
		if (state.right - state.left - wheels_diff >= desired_diff)
			break;
		usleep(1000);
	}
	//stop
	pthread_mutex_lock (&motor_msg_mutex);
   	 motor_msg.motor_left_speed  = MOTOR_STOP;
	motor_msg.motor_right_speed = MOTOR_STOP; 
	pthread_mutex_unlock (&motor_msg_mutex);
}

static void
scan_perimeter() {
	pthread_mutex_lock( &need_publish_mutex );
    need_publish = 1;
    pthread_mutex_unlock ( &need_publish_mutex );
}

void drive_in_a_rectangle(){
	for(int i = 0; i < 4; ++i){
		// move 2 inch twice and 3 inch twice, alternatively.
		drive_forward(2 + i%2);
		take_a_pic(state.pic++);
		scan_perimeter();
		turn_left();
	}
}

static void sig_handler(int signo)
{
    rp_lidar_stop(rp_run_state->dev);
}

static void* scan_loop()
{
    rp_state_t *state = rp_run_state;

    printf("Beginning scans...\n");
    if (system("echo 1 > /sys/class/gpio/gpio122/value")) {
      printf("Error starting rplidar motor\n");
    }
    // This loops forever, barring an error
    //rp_lidar_scan(state->dev, state->lcm, state->channel);
    rp_lidar_scan(state->dev, state->lcm, state->channel);
    printf("Terminating rplidar...\n");
    if (system("echo 0 > /sys/class/gpio/gpio122/value")) {
      printf("Error Stopping rplidar motor\n");
    }

    return NULL;
}

int init_rplidar(int argc, char** argv) {
	rp_run_state = (rp_state_t*) calloc(1, sizeof(rp_state_t));

    rp_run_state->gopt = getopt_create();
    getopt_add_bool(rp_run_state->gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string(rp_run_state->gopt, 'd', "device", "/dev/ttyO0", "Serial device");
    getopt_add_int(rp_run_state->gopt, 'b', "baud", "115200", "Baud rate");
    getopt_add_string(rp_run_state->gopt, 'c', "channel", "MAEBOT_LASER_SCAN", "LCM channel name");

    if (!getopt_parse(rp_run_state->gopt, argc, argv, 1) || getopt_get_bool(rp_run_state->gopt, "help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(rp_run_state->gopt);
        return 0;
    }

    signal(SIGTERM, sig_handler);
    signal(SIGINT, sig_handler);

    // Open device
    const char *name = getopt_get_string(rp_run_state->gopt, "device");
    int baud = getopt_get_int(rp_run_state->gopt, "baud");
    rp_run_state->dev = serial_open(name, baud, 1);

    if (rp_run_state->dev == -1) {
        printf("ERR: Could not open device at %s\n", name);
        return -1;
    }

    rp_run_state->lcm = lcm_create(NULL);
    rp_run_state->channel = getopt_get_string(rp_run_state->gopt, "channel");

    // Check device health
    if (rp_lidar_check_health(rp_run_state->dev) != HEALTH_GOOD)
        return -2;

    // Check device info
    if (VERBOSE)
        rp_lidar_check_info(rp_run_state->dev);

    // Begin scanning
    pthread_create(&rp_run_state->scan_thread, NULL, scan_loop, rp_run_state);
    return 0;
}


static void 
destroy_rplidar(){
	rp_lidar_stop(rp_run_state->dev);
	pthread_join(rp_run_state->scan_thread, NULL);

    lcm_destroy(rp_run_state->lcm);
    getopt_destroy(rp_run_state->gopt);
    serial_close(rp_run_state->dev);
    free(rp_run_state);
}

int main(int argc, char** argv){

	if (pthread_mutex_init (&motor_msg_mutex, NULL)) {
		printf ("motor mutex init failed\n");
		return 1;
	}

	// Init msg
	// no need for mutex here, as command thread hasn't started yet.
	motor_msg.motor_left_speed = MOTOR_STOP;
	motor_msg.motor_right_speed = MOTOR_STOP;

	//initiallize motor command thread
	pthread_t diff_drive_thread_pid;
   	pthread_create (&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);
	
	state.pic = 0;
	init_rplidar(argc, argv);
	
	for(int i = 0; i < 3; ++i){
		drive_in_a_rectangle();
	}

	pthread_mutex_lock (&motor_msg_mutex);
	motor_msg.motor_left_speed  = MOTOR_STOP;
	motor_msg.motor_right_speed = MOTOR_STOP; 
	pthread_mutex_unlock (&motor_msg_mutex);
	
	destroy_rplidar();

	usleep(1000000);

	return 0;
}







