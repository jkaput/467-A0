#include <lcm/lcm.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <pthread.h>
#include <unistd.h>

#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

#define PERIOD 50000 //us  -> 20Hz
#define MOTOR_SPEED 0.25f
#define MOTOR_STOP 0.0f

#define WHEEL_RADIUS 0.032f
#define WHEELBASE 0.08f
#define TICKS_REV 480
#define TICKS_FOOT 1456 

// thread to drive motors
maebot_motor_command_t motor_msg;
pthread_mutex_t motor_msg_mutex;

//start motor feedback to get ticks
/*lcm_t *lcm = lcm_create (NULL);

maebot_motor_feedback_t_subscribe (lcm,
                 	              	"MAEBOT_MOTOR_FEEDBACK",
								   	motor_feedback_handler,
									NULL);
*/
//keep track of ticks
typedef struct{
	int32_t left;
	int32_t right;
}State;

State state;



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
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");

    printf ("Subscribed to channel: %s\n", channel);
    printf ("utime: %"PRId64"\n", msg->utime);
    printf ("encoder_[left, right]_ticks:\t\t%d,\t%d\n",
            msg->encoder_left_ticks, msg->encoder_right_ticks);
    printf ("motor_current[left, right]:\t\t%d,\t%d\n",
            msg->motor_current_left, msg->motor_current_right);
    printf ("motor_[left, right]_commanded_speed:\t%f,\t%f\n",
            msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);
    printf ("motor_[left, right]_actual_speed:\t%f,\t%f\n",
            msg->motor_left_commanded_speed, msg->motor_right_commanded_speed);

	state.left = msg->encoder_left_ticks;
	state.right = msg->encoder_right_ticks;
}

/*
typedef struct{
	float x;
	float y;
	float theta;
}State;

State state = {0.0, 0.0, 0.0};
*/


void drive_forward(int dist){

	lcm_t *lcm = lcm_create (NULL);

	maebot_motor_feedback_t_subscribe (lcm,
                 	              	"MAEBOT_MOTOR_FEEDBACK",
								   	motor_feedback_handler,
									NULL);

	lcm_handle (lcm);
	usleep(1000000);
	//store value of ticks before moving
//	int32_t left_init = state.left;
//	int32_t right_init = state.right;

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
		
		if(diff < 0)
				diff = 0;

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

//	int32_t left_init = state.left;
//	int32_t target = state.left - 60;//doesnt make math sense
									//fudging to make work(sort of)

	//turn left
	pthread_mutex_lock (&motor_msg_mutex);
    motor_msg.motor_left_speed  = -MOTOR_SPEED * 0.75f;
	motor_msg.motor_right_speed = MOTOR_SPEED * 0.75f; 
	pthread_mutex_unlock (&motor_msg_mutex);
	
	//keep updating state while not met distance target
//	while(state.left > target)
//			lcm_handle (lcm);
	usleep(1000000 - 200);
	//stop
	pthread_mutex_lock (&motor_msg_mutex);
    motor_msg.motor_left_speed  = MOTOR_STOP;
	motor_msg.motor_right_speed = MOTOR_STOP; 
	pthread_mutex_unlock (&motor_msg_mutex);
	pthread_mutex_lock (&motor_msg_mutex);
    motor_msg.motor_left_speed  = MOTOR_STOP;
	motor_msg.motor_right_speed = MOTOR_STOP; 
	pthread_mutex_unlock (&motor_msg_mutex);

	lcm_handle(lcm);

}

void drive_in_a_rectangle(){

	for(int i = 0; i < 1; ++i){
		drive_forward(2);
		turn_left();
		drive_forward(3);
		turn_left();
		drive_forward(2);
		turn_left();
		drive_forward(3);
		turn_left();
	}
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
	
	//initialize motor feedback
/*	lcm_t *lcm = lcm_create (NULL);
	    if(!lcm)
			return 1;

	
	maebot_motor_feedback_t_subscribe (lcm,
                                       	"MAEBOT_MOTOR_FEEDBACK",
									   	motor_feedback_handler,
										NULL);*/
	drive_in_a_rectangle();

	pthread_mutex_lock (&motor_msg_mutex);
    motor_msg.motor_left_speed  = MOTOR_STOP;
	motor_msg.motor_right_speed = MOTOR_STOP; 
	pthread_mutex_unlock (&motor_msg_mutex);
	
	usleep(2000000);
	
	return 0;
}






