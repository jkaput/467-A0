#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>

#include <lcm/lcm.h>
#include "lcmtypes/maebot_sensor_data_t.h"

#include "../math/matd.h"

#define WHEEL_BASE 0.08f
#define WHEEL_DIAMETER 0.032f
#define GYRO_2_RADS 0.437372f
#define ACCEL_2_MET	0.000598f
#define PI 3.14159265f

typedef struct{
    matd_t* bot; // 3x2 state [x, Vx][y, Vy][theta, Vtheta]
	int64_t prev_time;
}State;
State state;


//might not need
typedef struct{
	int16_t gyro;
}IMU_State;

IMU_State imu_state;


float v_x_i = 0.0;
float v_y_i = 0.0;
float v_theta_i = 0.0;
float delta_x_local = 0.0;
float delta_y_local = 0.0;
float delta_x_global = 0.0;
float delta_y_global = 0.0;
float delta_theta = 0.0;
float delta_time = 0.0;
float delta_time_squared = 0.0;
float a_x = 0.0;
float a_y = 0.0;
float a_theta = 0.0;

static void
sensor_data_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                     const maebot_sensor_data_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");

    printf ("Subscribed to channel: MAEBOT_SENSOR_DATA\n");
    printf ("utime: %"PRId64"\n", msg->utime);
    printf ("accel[0, 1, 2]:        %d,\t%d,\t%d\n",
            msg->accel[0], msg->accel[1], msg->accel[2]);
    printf ("gyro[0, 1, 2]:         %d,\t%d,\t%d\n",
            msg->gyro[0], msg->gyro[1], msg->gyro[2]);
    printf ("gyro_int[0, 1, 2]:     %"PRId64",\t%"PRId64",\t%"PRId64"\n",
            msg->gyro_int[0], msg->gyro_int[1], msg->gyro_int[2]);
    printf ("line_sensors[0, 1, 2]: %d,\t%d,\t%d\n",
            msg->line_sensors[0], msg->line_sensors[1], msg->line_sensors[2]);
    printf ("range: %d\n", msg->range);
    printf ("user_button_pressed: %s\n", msg->user_button_pressed ? "true" : "false");
    printf ("power_button_pressed: %s\n", msg->power_button_pressed ? "true" : "false");

	//update state
	//x and y first
	if(state.prev_time == 0.0){
		state.prev_time = msg->utime;	
	}else{
		delta_time = msg->utime - state.prev_time;
		delta_time_squared = powf(delta_time, 2.0);
		state.prev_time = msg->utime;

		a_x = (float)msg->accel[0] * ACCEL_2_MET;
		a_y = (float)msg->accel[1] * ACCEL_2_MET;
		a_theta = (float)msg->gyro[2] * GYRO_2_RADS;
 
		//update x and y in state
		delta_x_local = matd_get(state.bot, 0, 1) * delta_time + 0.5 * a_x * delta_time_squared;
		delta_y_local = matd_get(state.bot, 1, 1) * delta_time + 0.5 * a_y * delta_time_squared;
		
		delta_x_global = delta_x_local * cosf(matd_get(state.bot, 2, 0)) +
						delta_y_local * cosf((PI/2) - matd_get(state.bot, 2, 0));
		delta_y_global = delta_x_local * sinf(matd_get(state.bot, 2, 0)) +
						delta_y_local * sinf((PI/2) - matd_get(state.bot, 2, 0));
		//update x, y state
		matd_put(state.bot, 0, 0, matd_get(state.bot, 0, 0) + delta_x_global);
		matd_put(state.bot, 1, 0, matd_get(state.bot, 1, 0) + delta_y_global);

		//get instantaneous velocity
		v_x_i = (float)msg->accel[0] * ACCEL_2_MET * delta_time;
		v_y_i = (float)msg->accel[1] * ACCEL_2_MET * delta_time;

		//update Vx, Vy state
		matd_put(state.bot, 0, 1, matd_get(state.bot, 0, 1) + v_x_i);
		matd_put(state.bot, 1, 1, matd_get(state.bot, 1, 1) + v_y_i);

		//update theta
		delta_theta = matd_get(state.bot, 2, 1) * delta_time + 0.5 * a_theta * delta_time_squared;

		matd_put(state.bot, 2, 0, matd_get(state.bot, 2, 0) + delta_theta);

		//update Vtheta
		v_theta_i = a_theta * delta_time;
		matd_put(state.bot, 2, 2, matd_get(state.bot, 2, 2) + v_theta_i);

	}
}

int
main (int argc, char *argv[])
{

	//instantiate state
	state.bot = matd_create(3,2); // [x][y][theta]
	state.prev_time = 0.0;
    
	lcm_t *lcm = lcm_create (NULL);
    if(!lcm)
        return 1;

    maebot_sensor_data_t_subscribe (lcm,
                                    "MAEBOT_SENSOR_DATA",
                                    sensor_data_handler,
                                    NULL);

    while (1)
        lcm_handle (lcm);

    return 0;

}
