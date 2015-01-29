#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <vector>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "common/timestamp.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcm_handlers.hpp"
#define BASE_LENGTH 0.08 // meters
#define METERS_PER_TICK 0.00020944 // meters
#define ADC_PER_METER_PER_SECOND_PER_SECOND 1670.13251784
#define ADC_PER_RADIANS_PER_SECOND 7505.74711621
//RMC - grid stuff
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include <math/point.hpp>
#define GRID_WIDTH_M 10
#define GRID_HEIGHT_M 10
#define CELL_WIDTH_M 0.5
#define SAMPLE_SPACING 0.005
#define FREE_ADJUST_FACTOR 1
#define OCCUPIED_ADJUST_FACTOR 3.5

static maebot_pose_handler pose_handler;
static maebot_laser_scan_handler laser_scan_handler;

//****TO DO*****//
//add makefile and other relevant files
//add occupancy grid
//lots of occupancy math
//movement/angle correction between maebot and grid
//lots of vx stuff 

typedef struct
{
	//lcm_t* lcm;
	pthread_mutex_t lcm_mutex;

	getopt_t* gopt;

	// // odometry
	// bool first_odo;
	// double odo_x_curr, odo_y_curr, odo_heading_curr;
	// int odo_left_tick, odo_right_tick;
	// std::vector<float> odo_points;
	// pthread_mutex_t odo_points_mutex;
	// pthread_mutex_t odo_curr_mutex;


	//pose
	double pose_x_curr, pose_y_curr, pose_heading_curr, pose_time;
	std::vector<float> pose_points;
	pthread_mutex_t pose_curr_mutex;
	pthread_mutex_t pose_points_mutex;


	// lidar
	std::vector<float> lidar_thetas;
	std::vector<int64_t> lidar_times;
	std::vector<float> lidar_ranges;
	std::vector<float> intensities; //uncomment if needed
	double num_ranges;
	bool scan_complete;
	pthread_mutex_t scans_mutex;

	
	int running;

	image_u32_t *img;

	vx_application_t app;

	vx_world_t * world;
	zhash_t * layers;

	pthread_mutex_t mutex; // for accessing the arrays
	pthread_t animate_thread;

	eecs467::OccupancyGrid grid;
} state_t;

state_t * state;

// void* lcm_handle_thread(void* arg) {
// 	while(1) {
// 		lcm_handle_timeout(state->lcm, 1000 /15);
// 	}

// 	return NULL;
// }


static void * pose_thread(void* arg){

	pthread_mutex_lock(&state->pose_curr_mutex);
	state->pose_x_curr = pose_handler.get_x_pos();
	state->pose_y_curr = pose_handler.get_y_pos();
	state->pose_heading_curr = pose_handler.get_theta();
	state->pose_time = pose_handler.get_timestamp();
	pthread_mutex_unlock(&state->pose_curr_mutex);


	pthread_mutex_lock(&state->pose_points_mutex);
	state->pose_points.push_back(state->pose_x_curr);
	state->pose_points.push_back(state->pose_y_curr);
	state->pose_points.push_back(0);
	state->pose_points.push_back(state->pose_x_curr);
	state->pose_points.push_back(state->pose_y_curr);
	state->pose_points.push_back(0);
	pthread_mutex_unlock(&state->pose_points_mutex);
}

static void * laser_scan_thread(void* arg) {

	pthread_mutex_lock(&state->scans_mutex);
	state->num_ranges = laser_scan_handler.get_num_ranges();

	state->lidar_ranges = laser_scan_handler.get_ranges();
	state->lidar_thetas = laser_scan_handler.get_thetas();
	state->lidar_times = laser_scan_handler.get_times();

	state->scan_complete = true;
	pthread_mutex_unlock(&state->scans_mutex);

}

static void update_grid()
{
	double mae_t = state->pose_time;
	double mae_x = state->pose_x_curr;
	double mae_y = state->pose_y_curr;
	double mae_th = state->pose_heading_curr;
	int num_rays = state->num_ranges;

	for (int ray = 0; ray < num_rays; ray++)
	{
		//interpolate (or extrapolate) mae_values

		double laser_t = state->lidar_times[ray];
		double laser_m = state->lidar_ranges[ray];
		double laser_th = state->lidar_thetas[ray];

		double cos_th = cos(mae_th + laser_th);
		double sin_th = cos(mae_th + laser_th);

		for (double sample_m = SAMPLE_SPACING; sample_m < laser_m; sample_m += SAMPLE_SPACING)
		{
			double sample_x = mae_x + (sample_m*cos_th);
			double sample_y = mae_y + (sample_m*sin_th);
			
			eecs467::Point<int> sample_cell = global_position_to_grid_cell(eecs467::Point<double>(sample_x, sample_y), state->grid);
			if (!state->grid.isCellInGrid(sample_cell.x, sample_cell.y)) { break; }

			int8_t odds = state->grid.logOdds(sample_cell.x, sample_cell.y);
			odds -= FREE_ADJUST_FACTOR;
			state->grid.setLogOdds(sample_cell.x, sample_cell.y, odds);
		}

		//positive adjust for ray termination cell
		double laser_term_x = mae_x + laser_m*cos_th;
		double laser_term_y = mae_y + laser_m*sin_th;
		eecs467::Point<int> laser_term_cell = global_position_to_grid_cell(eecs467::Point<double>(laser_term_x, laser_term_y), state->grid);
		int8_t odds = state->grid.logOdds(laser_term_cell.x, laser_term_cell.y);
		odds += OCCUPIED_ADJUST_FACTOR;
		state->grid.setLogOdds(laser_term_cell.x, laser_term_cell.y, odds);
	}
}

/* old a0 odometry unnecessary for task 1
static void motor_feedback_handler(const lcm_recv_buf_t *rbuf,
	const char *channel,
	const maebot_motor_feedback_t *msg, void *user)
{
	if (state->first_odo) {
		state->odo_left_tick = msg->encoder_left_ticks;
		state->odo_right_tick = msg->encoder_right_ticks;
		state->first_odo = false;
		return;
	}

	double deltaLeft = msg->encoder_left_ticks - state->odo_left_tick;
	double deltaRight = msg->encoder_right_ticks - state->odo_right_tick;
	state->odo_left_tick = msg->encoder_left_ticks;
	state->odo_right_tick = msg->encoder_right_ticks;

	deltaLeft *= METERS_PER_TICK;
	deltaRight *= METERS_PER_TICK;

	double distance = (deltaRight + deltaLeft) / 2;
	double theta = (deltaRight - deltaLeft) / BASE_LENGTH;
	double alpha = theta / 2;
	pthread_mutex_lock(&state->odo_curr_mutex);
	state->odo_x_curr += distance * cos(state->odo_heading_curr + alpha);
	state->odo_y_curr += distance * sin(state->odo_heading_curr + alpha);
	pthread_mutex_unlock(&state->odo_curr_mutex);
	state->odo_heading_curr += theta;

	pthread_mutex_lock(&state->odo_points_mutex);
	pthread_mutex_lock(&state->odo_curr_mutex);
	state->odo_points.push_back(state->odo_x_curr);
	state->odo_points.push_back(state->odo_y_curr);
	state->odo_points.push_back(0);
	state->odo_points.push_back(state->odo_x_curr);
	state->odo_points.push_back(state->odo_y_curr);
	state->odo_points.push_back(0);
	pthread_mutex_unlock(&state->odo_curr_mutex);
	pthread_mutex_unlock(&state->odo_points_mutex);
}
*/

void* draw_thread(void*) {
	while (1) {
		int vec_size;
		vx_resc_t* verts;
		// drawing odometry path
		// if (getopt_get_bool(state->gopt, "paths")) {
		// 	pthread_mutex_lock(&state->odo_points_mutex);
		// 	vec_size = state->odo_points.size();
		// 	verts = vx_resc_copyf((state->odo_points).data(), vec_size);
		// 	pthread_mutex_unlock(&state->odo_points_mutex);
		// 	vx_buffer_add_back(vx_world_get_buffer(state->world,"draw_thread"),
		// 		vxo_lines(verts, vec_size / 3, GL_LINES, vxo_lines_style(vx_red, 2.0f)));

		
		// }

		// if (getopt_get_bool(state->gopt, "scans")) {
		// 	pthread_mutex_lock(&state->scans_mutex);
		// 	for (unsigned int i = 0; i < state->scans_lidar.size(); ++i) {
		// 		vec_size = state->scans_lidar[i].size();
		// 		// printf("vec_size: %d\n", vec_size);
		// 		verts = vx_resc_copyf((state->scans_lidar[i]).data(), vec_size);
		// 		vx_buffer_add_back(vx_world_get_buffer(state->world,"draw_thread"),
		// 			vxo_lines(verts, vec_size / 3, GL_LINES,
		// 				vxo_lines_style(vx_green, 2.0f)));
		// 	}
		// 	pthread_mutex_unlock(&state->scans_mutex);

		// 	for (unsigned int i = 0; i < state->images.size(); ++i) {
		// 		image_u32_t* image = state->images[i];
		// 		vx_object_t* obj = vxo_image_texflags(vx_resc_copyui(image->buf,
		// 			image->stride*image->height),
		// 			image->width, image->height, image->stride,
		// 			GL_RGBA, VXO_IMAGE_FLIPY,
		// 			VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

		// 		int x_loc = -2250 + (i * 1500);
		// 		int y_loc = -2000 - image->height;
		// 		vx_buffer_add_back(vx_world_get_buffer(state->world,"draw_thread"),
		// 			vxo_chain(vxo_mat_scale(1.0/image->height),
		// 			vxo_mat_translate3(x_loc, y_loc, 0), obj));
		// 	}
		// }

		vx_buffer_swap(vx_world_get_buffer(state->world,"draw_thread"));
		usleep(1000);
	}

	return NULL;
}

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
	state_t * state = (state_t *) app->impl;
	pthread_mutex_lock(&state->mutex);

	vx_layer_t * layer = NULL;

	// store a reference to the world and layer that we associate with each vx_display_t
	zhash_remove(state->layers, &disp, NULL, &layer);

	vx_layer_destroy(layer);

	pthread_mutex_unlock(&state->mutex);
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
	state_t * state = (state_t *) app->impl;

	vx_layer_t * layer = vx_layer_create(state->world);
	vx_layer_set_display(layer, disp);

	pthread_mutex_lock(&state->mutex);
	// store a reference to the world and layer that we associate with each vx_display_t
	zhash_put(state->layers, &disp, &layer, NULL, NULL);
	pthread_mutex_unlock(&state->mutex);
}

static void state_destroy(state_t * state)
{

	if (state->img != NULL)
		image_u32_destroy(state->img);

	vx_world_destroy(state->world);
	assert(zhash_size(state->layers) == 0);

	zhash_destroy(state->layers);
	free(state);

	pthread_mutex_destroy(&state->mutex);
	pthread_mutex_destroy(&state->lcm_mutex);
	//pthread_mutex_destroy(&state->odo_points_mutex);
	pthread_mutex_destroy(&state->scans_mutex);
	//pthread_mutex_destroy(&state->odo_curr_mutex);
	//lcm_destroy(state->lcm);

	getopt_destroy(state->gopt);
}

static state_t * state_create()
{
	state_t * state = (state_t * ) calloc(1, sizeof(state_t));
	state->running = 1;
	state->app.impl=state;
	state->app.display_started=display_started;
	state->app.display_finished=display_finished;


	if (pthread_mutex_init(&state->lcm_mutex, NULL)) {
		printf("lcm mutex init failed\n");
		exit(1);
	}

	// if (pthread_mutex_init(&state->odo_points_mutex, NULL)) {
	// 	printf("odo points mutex init failed\n");
	// 	exit(1);
	// }


	if (pthread_mutex_init(&state->scans_mutex, NULL)) {
		printf("scans mutex init failed\n");
		exit(1);
	}

	// if (pthread_mutex_init(&state->odo_curr_mutex, NULL)) {
	// 	printf("odo curr mutex init failed\n");
	// 	exit(1);
	// }

	//state->first_odo = true;
	state->pose_y_curr = 0;
	state->pose_x_curr = 0;
	state->pose_heading_curr = 0;


	state->gopt = getopt_create();
	getopt_add_bool(state->gopt, 's', "scans", 0, "shows scans");
	getopt_add_bool(state->gopt, 'p', "paths", 0, "shows paths");

	state->world = vx_world_create();
	state->layers = zhash_create(sizeof(vx_display_t*),
		sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

	pthread_mutex_init (&state->mutex, NULL);

	return state;
}

int main(int argc, char ** argv)
{
	vx_global_init(); // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library

	state = state_create();
	if (!getopt_parse(state->gopt, argc, argv, 1)) {
		printf("couldn't parse getopt\n");
		exit(1);
	}

	// for (int i = 0; i < 4; ++i) {
	// 	char count_str[10];
	// 	sprintf(count_str, "%d", i);
	// 	char path[100] = "pictures/corner_image_";
	// 	strcat(path, count_str);
	// 	strcat(path, ".ppm");
	// 	image_u32_t* image = image_u32_create_from_pnm(path);
	// 	if (image == NULL) {
	// 		printf("image %s unsuccesfully loaded\n", path);
	// 		exit(1);
	// 	}
	// 	state->images.push_back(image);
	// }

	pthread_t draw_thread_pid;
	pthread_create(&draw_thread_pid, NULL, draw_thread, NULL);

	gdk_threads_init();
	gdk_threads_enter();
	gtk_init(&argc, &argv);

	vx_gtk_display_source_t * appwrap = vx_gtk_display_source_create(&state->app);
	GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	GtkWidget * canvas = vx_gtk_display_source_get_widget(appwrap);
	gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
	gtk_container_add(GTK_CONTAINER(window), canvas);
	gtk_widget_show (window);
	gtk_widget_show (canvas); // XXX Show all causes errors!

	g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

	lcm::LCM lcm;
    if(!lcm.good())
        return 1;



	lcm.subscribe("MAEBOT_LASER_SCAN", &maebot_laser_scan_handler::handleMessage, &laser_scan_handler);

	lcm.subscribe("MAEBOT_POSE", &maebot_pose_handler::handleMessage, &pose_handler);

	pthread_t laser_thread_pid;
	pthread_create(&laser_thread_pid, NULL, laser_scan_thread, NULL);

	pthread_t pose_thread_pid;
	pthread_create(&pose_thread_pid, NULL, pose_thread, NULL);

	// pthread_t lcm_handle_thread_pid;
	// pthread_create(&lcm_handle_thread_pid, NULL, lcm_handle_thread, NULL);

	// maebot_motor_feedback_t_subscribe(state->lcm,
	// 	"MAEBOT_MOTOR_FEEDBACK",
	// 	motor_feedback_handler,
	// 	NULL);


	printf("All subscribed\n");

	gtk_main(); // Blocks as long as GTK window is open
	gdk_threads_leave();

	vx_gtk_display_source_destroy(appwrap);

	state_destroy(state);
	vx_global_destroy();
}
