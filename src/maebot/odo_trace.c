#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>

#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"
#include "../math/matd.h"

#include <gtk/gtk.h>
#include "vx/vxo_drawables.h"
#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_global.h"
#include "vx/vx_layer.h"
#include "vx/vx_world.h"
#include "vx/vx_colors.h"


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
  lcm_t *motor_lcm;
  lcm_t *lidar_lcm;

	matd_t* bot; // 3x1 state [x][y][theta]
} State;
State state;

//previous odometry reading
typedef struct{
	int32_t left;
	int32_t right;
	int8_t init;
} Odo_State;
Odo_State odo_state = {0, 0, 0};

typedef struct{
  vx_object_t *obj;
  char *name;
} obj_data_t;

typedef struct{
  zarray_t *obj_data;
  vx_world_t *world;
} vx_state_t;
vx_state_t vx_state;

float delta_x;
float delta_y;
float delta_theta;

#define  ADD_OBJECT(s, call)					    \
  {								    \
    obj_data_t data = {.obj = s call, .name = #s};		    \
    zarray_add(vx_state.obj_data, &data);				    \
  }  


static void draw(vx_world_t * world, zarray_t * obj_data);
static void display_finished(vx_application_t * app, vx_display_t * disp);
static void display_started(vx_application_t * app, vx_display_t * disp);

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
                        const maebot_motor_feedback_t *msg, void *user)
{
    int res = system ("clear");
    if (res)
        printf ("system clear failed\n");
    printf("Handling motor\n");
/*  //print a bunch of info 
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
/*
	//update state
	if(!odo_state.init){
		odo_state.left = msg->encoder_left_ticks;
		odo_state.right = msg->encoder_right_ticks;
		odo_state.init = 1;
	} else{
		delta_left = msg->encoder_left_ticks - odo_state.left;
		delta_right = msg->encoder_right_ticks - odo_state.right;
	}//end else
*/

}

static void
rplidar_feedback_handler(const lcm_recv_buf_t *rbuf, const char *channel,
		       const maebot_laser_scan_t *scan, void *user)
{
  printf("Handling rplidar\n");

  //ADD_OBJECT(vxo_line, (vxo_mesh_style(vx_green)));
  int i, npoints;
  float single_line[6]; // x1, y1, z1, x2, y2, z2
  // need to:
  //    center around maebot's position

  //    convert to x,y

  //    plot lines
  npoints = 2;
  single_line[0] = /*maebot starting x*/ 0.0;
  single_line[1] = /*maebot starting y*/ 0.0;
  single_line[2] = /*maebot starting z*/ 0.0;

  vx_buffer_t *mybuf = vx_world_get_buffer(vx_state.world, "mybuf");
  

  for(i = 0; i < scan->num_ranges; ++i){
    // currently centered around origin, will need to be centered around maebot position
    single_line[3] = (scan->ranges[i]) * cos(scan->thetas[i]);
    single_line[4] = (scan->ranges[i]) * sin(scan->thetas[i]);
    single_line[5] = 0.0;

    vx_resc_t *verts = vx_resc_copyf(single_line, npoints*3);
    //ADD_OBJECT(vxo_lines, (verts, npoints, GL_LINES, vxo_points_style(vx_green, 2.0f)));
    vx_buffer_add_back(mybuf, vxo_lines(verts, npoints, GL_LINES, vxo_points_style(vx_green, 2.0f)));
    //single_line[0] = single_line[3];
    //single_line[1] = single_line[4];
    //single_line[2] = single_line[5];
  }
    vx_buffer_swap(mybuf);
}

static void*
lcm_thread_handler(void *args)
{
  State *lcm_state = (State*) args;
  while(1){
    //lcm_handle (lcm_state->motor_lcm);
    printf("Looping in lcm_thread_handler\n");
    lcm_handle (lcm_state->lidar_lcm);
  }
  return NULL;
}

int
main (int argc, char *argv[])
{
  vx_global_init();
  
  vx_state.world = vx_world_create();
  vx_state.obj_data = zarray_create(sizeof(obj_data_t));

  // ADD_OBJECT(obj_type, args);

  vx_application_t app = {.impl=&vx_state, .display_started = display_started, .display_finished = display_finished};

  gdk_threads_init();
  gdk_threads_enter();
  gtk_init(&argc, &argv);

	state.bot = matd_create(3,1); // [x][y][theta]
	

	state.motor_lcm = lcm_create (NULL);
	if(!state.motor_lcm)
	  return 1;

	state.lidar_lcm = lcm_create(NULL);
	if(!state.lidar_lcm)
	  return 1;

	//printf ("utime,\t\tleft_ticks,\tright_ticks\n");
	/*
	maebot_motor_feedback_t_subscribe (state.motor_lcm,
                        	           "MAEBOT_MOTOR_FEEDBACK",
                	                   motor_feedback_handler,
        	                           NULL);
	*/
	maebot_laser_scan_t_subscribe (state.lidar_lcm,
				       "MAEBOT_LASER_SCAN",
				       rplidar_feedback_handler,
				       NULL);
	
	pthread_t lcm_handler_thread;
	pthread_create(&lcm_handler_thread, NULL, lcm_thread_handler, (void*)(&state));

    vx_gtk_display_source_t * appwrap = vx_gtk_display_source_create(&app);
    GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas = vx_gtk_display_source_get_widget(appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window), 1280, 720);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show (window);
    gtk_widget_show (canvas); // XXX Show all causes errors!

    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_main (); // Blocks as long as GTK window is open
    gdk_threads_leave ();

    vx_gtk_display_source_destroy(appwrap);
    vx_global_destroy();

	return 0;
}

static void draw(vx_world_t * world, zarray_t * obj_data)
{
    vx_buffer_t * vb = vx_world_get_buffer(world, "zoo");

    int cols = sqrt(zarray_size(obj_data)) + 1;
    int rows = zarray_size(obj_data)/cols + 1;
    int grid = 4;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            int idx = y*cols + x;
            if (idx >= zarray_size(obj_data))
                break;

            obj_data_t  data;
            zarray_get(obj_data, idx, &data);
            vx_object_t * vo = data.obj;

            vx_buffer_add_back(vb, vxo_chain(vxo_mat_translate2(x*grid + grid/2, rows*grid - (y*grid +grid/2)),
                                             vxo_chain(vxo_mat_scale(grid),
                                                       vxo_rect(vxo_lines_style(vx_gray, 2))),
                                             vo));
            // XXX Text, box outlines
        }
    }
    vx_buffer_swap(vb); // XXX Could name buffers by object name
}

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
    // XXX layer leak
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
    vx_state_t * VX_state = app->impl;

    vx_layer_t * layer = vx_layer_create(VX_state->world);
    vx_layer_set_display(layer, disp);
    vx_layer_set_background_color(layer, vx_black);
    // XXX bug in world
    draw(VX_state->world, VX_state->obj_data);
}

