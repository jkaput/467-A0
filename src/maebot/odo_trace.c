#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "../math/matd.h"

//might not 
/*
typedef struct {
	float x;
	float y;
	float theta;
	int32_t left;
	int32_t right;
	int32_t prev_left;
	int32_t prev_right;
	int8_t init;
}State;

State state = {0.0, 0.0, 0.0, 0, 0, 0, 0, 0}; 
*/

typedef struct{
	matd_t* bot; // 3x1 state [x][y][theta]
} State;

State state;

typedef struct{
	int32_t left;
	int32_t right;
	int8_t init;
} Odo_State;

Odo_State odo_state = {0, 0, 0};



float delta_x;
float delta_y;
float delta_theta;

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const maebot_motor_feedback_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");
/*
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
*/

	//update state
	if(odo_state.init){
		odo_state.left = msg->encoder_left_ticks;
		odo_state.right = msg->encoder_right_ticks;
		state.init = 1;
	} else{
		
	}//end else


}

int
main (int argc, char *argv[])
{
	state.bot = matd_create(3,1); // [x][y][theta]
	
	delta_x = 0.0;
	delta_y = 0.0;
	delta_theta = 0.0;

	lcm_t *lcm = lcm_create (NULL);
	if(!lcm)
		return 1;

	printf ("utime,\t\tleft_ticks,\tright_ticks\n");

	maebot_motor_feedback_t_subscribe (lcm,
                        	           "MAEBOT_MOTOR_FEEDBACK",
                	                   motor_feedback_handler,
        	                           NULL);

	while (1)
		lcm_handle (lcm);

	return 0;
}
