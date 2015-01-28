#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm.h>

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
#include "lcmtypes/maebot_a0_corner_scan_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"

#define BASE_LENGTH 0.08 // meters
#define METERS_PER_TICK 0.00020944 // meters
#define ADC_PER_METER_PER_SECOND_PER_SECOND 1670.13251784
#define ADC_PER_RADIANS_PER_SECOND 7505.74711621

typedef struct
{
	lcm_t* lcm;
	pthread_mutex_t lcm_mutex;

	getopt_t* gopt;

	// odometry
	bool first_odo;
	double odo_x_curr, odo_y_curr, odo_heading_curr;
	int odo_left_tick, odo_right_tick;
	std::vector<float> odo_points;
	pthread_mutex_t odo_points_mutex;
	pthread_mutex_t odo_curr_mutex;


	// lidar
	std::vector<std::vector<float>> scans_lidar;
	pthread_mutex_t scans_mutex;

	
	int running;

	image_u32_t *img;

	vx_application_t app;

	vx_world_t * world;
	zhash_t * layers;

	pthread_mutex_t mutex; // for accessing the arrays
	pthread_t animate_thread;
} state_t;

state_t * state;

void* lcm_handle_thread(void* arg) {
	while(1) {
		lcm_handle_timeout(state->lcm, 1000 /15);
	}

	return NULL;
}


static void corner_scan_handler(const lcm_recv_buf_t* rbuf,
	const char* channel,
	const maebot_a0_corner_scan_t *msg, void* user) {
	if (state->scans_lidar.size() >= 4) {
		return;
	}

	std::vector<float> new_lidar_scan;
	// bool outward = true;
	pthread_mutex_lock(&state->odo_curr_mutex);
	double odo_x_curr = state->odo_x_curr;
	double odo_y_curr = state->odo_y_curr;
	pthread_mutex_unlock(&state->odo_curr_mutex);

	for (int i = 0; i < msg->lidar.num_ranges; ++i) {
		float theta = msg->lidar.thetas[i];
		float range = msg->lidar.ranges[i];
		float new_pt_x = odo_x_curr + range * cos(theta);
		float new_pt_y = odo_y_curr + range * sin(theta);
		new_lidar_scan.push_back(odo_x_curr);
		new_lidar_scan.push_back(odo_y_curr);
		new_lidar_scan.push_back(-0.001);

		new_lidar_scan.push_back(new_pt_x);
		new_lidar_scan.push_back(new_pt_y);
		new_lidar_scan.push_back(-0.001);
	}

	pthread_mutex_lock(&state->scans_mutex);
	state->scans_lidar.push_back(new_lidar_scan);
	pthread_mutex_unlock(&state->scans_mutex);
}

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

void* draw_thread(void*) {
	while (1) {
		int vec_size;
		vx_resc_t* verts;
		// drawing odometry path
		if (getopt_get_bool(state->gopt, "paths")) {
			pthread_mutex_lock(&state->odo_points_mutex);
			vec_size = state->odo_points.size();
			verts = vx_resc_copyf((state->odo_points).data(), vec_size);
			pthread_mutex_unlock(&state->odo_points_mutex);
			vx_buffer_add_back(vx_world_get_buffer(state->world,"draw_thread"),
				vxo_lines(verts, vec_size / 3, GL_LINES, vxo_lines_style(vx_red, 2.0f)));

		
		}

		if (getopt_get_bool(state->gopt, "scans")) {
			pthread_mutex_lock(&state->scans_mutex);
			for (unsigned int i = 0; i < state->scans_lidar.size(); ++i) {
				vec_size = state->scans_lidar[i].size();
				// printf("vec_size: %d\n", vec_size);
				verts = vx_resc_copyf((state->scans_lidar[i]).data(), vec_size);
				vx_buffer_add_back(vx_world_get_buffer(state->world,"draw_thread"),
					vxo_lines(verts, vec_size / 3, GL_LINES,
						vxo_lines_style(vx_green, 2.0f)));
			}
			pthread_mutex_unlock(&state->scans_mutex);

			for (unsigned int i = 0; i < state->images.size(); ++i) {
				image_u32_t* image = state->images[i];
				vx_object_t* obj = vxo_image_texflags(vx_resc_copyui(image->buf,
					image->stride*image->height),
					image->width, image->height, image->stride,
					GL_RGBA, VXO_IMAGE_FLIPY,
					VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

				int x_loc = -2250 + (i * 1500);
				int y_loc = -2000 - image->height;
				vx_buffer_add_back(vx_world_get_buffer(state->world,"draw_thread"),
					vxo_chain(vxo_mat_scale(1.0/image->height),
					vxo_mat_translate3(x_loc, y_loc, 0), obj));
			}
		}

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
	pthread_mutex_destroy(&state->odo_points_mutex);
	pthread_mutex_destroy(&state->imu_points_mutex);
	pthread_mutex_destroy(&state->scans_mutex);
	pthread_mutex_destroy(&state->odo_curr_mutex);
	lcm_destroy(state->lcm);

	getopt_destroy(state->gopt);
}

static state_t * state_create()
{
	state_t * state = (state_t * ) calloc(1, sizeof(state_t));
	state->running = 1;
	state->app.impl=state;
	state->app.display_started=display_started;
	state->app.display_finished=display_finished;

	state->lcm = lcm_create(NULL);
	if (!state->lcm) {
		printf("lcm create failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state->lcm_mutex, NULL)) {
		printf("lcm mutex init failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state->odo_points_mutex, NULL)) {
		printf("odo points mutex init failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state->imu_points_mutex, NULL)) {
		printf("imu points mutex init failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state->scans_mutex, NULL)) {
		printf("scans mutex init failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state->odo_curr_mutex, NULL)) {
		printf("odo curr mutex init failed\n");
		exit(1);
	}

	state->first_odo = true;
	state->odo_y_curr = 0;
	state->odo_x_curr = 0;
	state->odo_heading_curr = 0;
	state->odo_left_tick = 0;
	state->odo_right_tick = 0;
	state->odo_points.push_back(0);
	state->odo_points.push_back(0);
	state->odo_points.push_back(0);


	state->imu_first = true;
	state->imu_vel_x_curr = 0;
	state->imu_vel_y_curr = 0;
	state->imu_pos_x_curr = 0;
	state->imu_pos_y_curr = 0;
	state->imu_heading_curr = 0;
	state->imu_utime_last = 0;
	state->imu_points.push_back(0);
	state->imu_points.push_back(0);
	state->imu_points.push_back(0);

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

	for (int i = 0; i < 4; ++i) {
		char count_str[10];
		sprintf(count_str, "%d", i);
		char path[100] = "pictures/corner_image_";
		strcat(path, count_str);
		strcat(path, ".ppm");
		image_u32_t* image = image_u32_create_from_pnm(path);
		if (image == NULL) {
			printf("image %s unsuccesfully loaded\n", path);
			exit(1);
		}
		state->images.push_back(image);
	}

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


	maebot_motor_feedback_t_subscribe(state->lcm,
		"MAEBOT_MOTOR_FEEDBACK",
		motor_feedback_handler,
		NULL);

	maebot_sensor_data_t_subscribe(state->lcm,
		"MAEBOT_SENSOR_DATA",
		sensor_data_handler,
		NULL);

	maebot_a0_corner_scan_t_subscribe(state->lcm,
		"MAEBOT_A0_CORNER_SCAN",
		corner_scan_handler,
		NULL);

	pthread_t lcm_handle_thread_pid;
	pthread_create(&lcm_handle_thread_pid, NULL, lcm_handle_thread, NULL);

	printf("All subscribed\n");

	gtk_main(); // Blocks as long as GTK window is open
	gdk_threads_leave();

	vx_gtk_display_source_destroy(appwrap);

	state_destroy(state);
	vx_global_destroy();
}
