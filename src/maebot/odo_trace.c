#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>

#include <lcm/lcm.h>
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"
#include "../math/matd.h"
#include "../math/fasttrig.h"

#include <gtk/gtk.h>
#include "vx/vxo_drawables.h"
#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_global.h"
#include "vx/vx_layer.h"
#include "vx/vx_world.h"
#include "vx/vx_colors.h"

#define WHEEL_BASE 0.08f
#define WHEEL_DIAMETER 0.032f
#define DISTANCE_TICK 0.0002094f

typedef struct{
  lcm_t *motor_lcm;
  lcm_t *lidar_lcm;

  matd_t* bot; // 3x1 state [x][y][theta]
  char buffer_name[8];
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

int32_t delta_left;
int32_t delta_right;
float delta_s;
float delta_s_left;
float delta_s_right;
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
	
	//update state
	if(!odo_state.init){
		odo_state.left = msg->encoder_left_ticks;
		odo_state.right = msg->encoder_right_ticks;
		odo_state.init = 1;
	} else{
		delta_left = msg->encoder_left_ticks - odo_state.left;
		delta_right = msg->encoder_right_ticks - odo_state.right;
		delta_s_left = DISTANCE_TICK * delta_left;
		delta_s_right = DISTANCE_TICK * delta_right;
		delta_s =((float)(delta_s_left + delta_s_right))/2.0;
		delta_theta = ((float)(delta_s_right - delta_s_left))/2.0 + matd_get(state.bot, 2, 0);
		matd_put(state.bot, 2, 0, delta_theta);
		
		delta_x = abs(delta_s)*cosf((float)delta_theta) + matd_get(state.bot, 0, 0);
		matd_put(state.bot, 0, 0, delta_x);

		delta_y = abs(delta_s)*sinf((float)delta_theta) + matd_get(state.bot, 1, 0);
		matd_put(state.bot, 1, 0, delta_y);

		odo_state.left = msg->encoder_left_ticks;
		odo_state.right = msg->encoder_right_ticks;
		
		
		printf("%f\t%f\t%f\n", delta_x, delta_y, delta_theta);

		float pt[3] = {matd_get(state.bot, 0, 0), matd_get(state.bot, 1, 0), 0.0};
		state.buffer_name[6]++;
		vx_resc_t *one_point = vx_resc_copyf(pt,3);
		vx_buffer_t *buf = vx_world_get_buffer(vx_state.world, state.buffer_name);
		vx_object_t *trace = vxo_chain(vxo_points(one_point, 1, vxo_points_style(vx_red, 2.0f)));
		vx_buffer_add_back(buf, trace);
		vx_buffer_swap(buf);
	}//end else
}

static void
rplidar_feedback_handler(const lcm_recv_buf_t *rbuf, const char *channel,
		       const maebot_laser_scan_t *scan, void *user)
{
  printf("Handling rplidar\n");

  //ADD_OBJECT(vxo_line, (vxo_mesh_style(vx_green)));
  int i, npoints;
  float single_line[6]; // x1, y1, z1, x2, y2, z2

  npoints = 2;
  single_line[0] = /*maebot starting x*/ 0.0;
  single_line[1] = /*maebot starting y*/ 0.0;
  single_line[2] = /*maebot starting z*/ 0.0;

  state.buffer_name[6]++;

  vx_buffer_t *mybuf = vx_world_get_buffer(vx_state.world, state.buffer_name);
  

  for(i = 0; i < scan->num_ranges; ++i){
    // currently centered around origin, will need to be centered around maebot position
    single_line[3] = (scan->ranges[i]) * cos(scan->thetas[i]);
    single_line[4] = (scan->ranges[i]) * sin(scan->thetas[i]);
    single_line[5] = 0.0;

    vx_resc_t *verts = vx_resc_copyf(single_line, npoints*3);
    /*
    vx_object_t *line = vxo_chain(vxo_mat_rotate_z(matd_get(state.bot,0,2)),
				  vxo_mat_translate3(matd_get(state.bot,0,0),matd_get(state.bot,0,1), 0.0),
				  vxo_lines(verts, npoints, GL_LINES, vxo_points_style(vx_green, 2.0f)));
    */
    vx_object_t *line = vxo_lines(verts, npoints, GL_LINES, vxo_points_style(vx_green, 2.0f));
    vx_buffer_add_back(mybuf, line);
  }
    vx_buffer_swap(mybuf);
}

static void*
lcm_lidar_handler(void *args)
{
  State *lcm_state = (State*) args;
  while(1){
    lcm_handle (lcm_state->lidar_lcm);
  }
  return NULL;
}

static void*
lcm_motor_handler(void *args)
{
  State *lcm_state = (State*) args;
  while(1){
    lcm_handle(lcm_state->motor_lcm);
  }
  return NULL;
}

int
main (int argc, char *argv[])
{

		fasttrig_init();
  vx_global_init();
  
  vx_state.world = vx_world_create();
  vx_state.obj_data = zarray_create(sizeof(obj_data_t));
  strcpy(state.buffer_name, "buffer0");
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

	maebot_motor_feedback_t_subscribe (state.motor_lcm,
                        	           "MAEBOT_MOTOR_FEEDBACK",
                	                   motor_feedback_handler,
        	                           NULL);

	maebot_laser_scan_t_subscribe (state.lidar_lcm,
				       "MAEBOT_LASER_SCAN",
				       rplidar_feedback_handler,
				       NULL);
	
	pthread_t lcm_lidar_thread, lcm_motor_thread;
	pthread_create(&lcm_motor_thread, NULL, lcm_motor_handler, (void*)(&state));
	pthread_create(&lcm_lidar_thread, NULL, lcm_lidar_handler, (void*)(&state));

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

