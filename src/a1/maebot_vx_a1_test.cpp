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
#include "maebot_data.hpp"
//#define BASE_LENGTH 0.08 // meters
//#define METERS_PER_TICK 0.00020944 // meters
//#define ADC_PER_METER_PER_SECOND_PER_SECOND 1670.13251784
//define ADC_PER_RADIANS_PER_SECOND 7505.74711621
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

//static maebot_pose_handler pose_handler;
//static maebot_laser_scan_handler laser_scan_handler;

//****TO DO*****//
//add makefile and other relevant files
//add occupancy grid
//lots of occupancy math
//movement/angle correction between maebot and grid
//lots of vx stuff 

typedef struct
{
    lcm::LCM lcm;
    pthread_mutex_t run_mutex;
	getopt_t* gopt;

	//pose
    maebot_pose_handler pose_handler;
	//double pose_x_curr, pose_y_curr, pose_heading_curr, pose_time;
	maebot_pose_data pose_curr;
    std::vector<maebot_pose_data> pose_data;
	pthread_mutex_t pose_curr_mutex;
	pthread_mutex_t pose_points_mutex;


	// lidar
    maebot_laser_scan_handler laser_scan_handler;
	//std::vector<float> lidar_thetas;
	//std::vector<int64_t> lidar_times;
	//std::vector<float> lidar_ranges;
	//std::vector<float> intensities; //uncomment if needed
	//double num_ranges;
    maebot_laser_data lidar_curr;
	bool scan_complete;
	pthread_mutex_t scans_mutex;

    // vx stuff	
	int running;
	image_u32_t *img;
	vx_application_t app;
	vx_world_t * world;
	zhash_t * layers;
	pthread_mutex_t mutex; // for accessing the arrays
	pthread_t animate_thread;
    //grid
	eecs467::OccupancyGrid grid;
} state_t;

state_t * state;
FILE *fp_pose;
void* run_lcm(void *input){
    //pthread_mutex_lock(&(state->run_mutex));
    while(1){
        //pthread_mutex_unlock(&(state->run_mutex));
        state->lcm.handle();
        //pthread_mutex_lock(&(state->run_mutex));
    }
    //pthread_mutex_unlock(&(state->run_mutex));
    return NULL;
}

static void * pose_thread(void* arg){
	pthread_mutex_lock(&state->pose_curr_mutex);
	//state->pose_x_curr = state->pose_handler.get_x_pos();
	//state->pose_y_curr = state->pose_handler.get_y_pos();
	//state->pose_heading_curr = state->pose_handler.get_theta();
	//state->pose_time = state->pose_handler.get_timestamp();
    state->pose_curr = maebot_pose_data(state->pose_handler.get_timestamp(),
            state->pose_handler.get_x_pos(),
            state->pose_handler.get_y_pos(),
            state->pose_handler.get_theta());
	pthread_mutex_unlock(&state->pose_curr_mutex);


	pthread_mutex_lock(&state->pose_points_mutex);
	//state->pose_points.push_back(state->pose_x_curr);
	//state->pose_points.push_back(state->pose_y_curr);
	//state->pose_points.push_back(0);
	//state->pose_points.push_back(state->pose_x_curr);
	//state->pose_points.push_back(state->pose_y_curr);
	//state->pose_points.push_back(0);
    state->pose_data.push_back(state->pose_curr);
	pthread_mutex_unlock(&state->pose_points_mutex);
    fprintf(fp_pose,"%f %f %f\n",state->pose_handler.get_x_pos(),state->pose_handler.get_y_pos(),state->pose_handler.get_theta());
}

static void * laser_scan_thread(void* arg) {

	pthread_mutex_lock(&state->scans_mutex);
	//state->num_ranges = state->laser_scan_handler.get_num_ranges();

	//state->lidar_ranges = state->laser_scan_handler.get_ranges();
	//state->lidar_thetas = state->laser_scan_handler.get_thetas();
	//state->lidar_times = state->laser_scan_handler.get_times();
	state->lidar_curr = maebot_laser_data(state->laser_scan_handler.get_timestamp(),
           state->laser_scan_handler.get_num_ranges(),
           state->laser_scan_handler.get_ranges(),
           state->laser_scan_handler.get_thetas(),
           state->laser_scan_handler.get_times(),
           state->laser_scan_handler.get_intensities(),
           state->pose_curr);
    state->scan_complete = true;
	pthread_mutex_unlock(&state->scans_mutex);

}

static void draw(state_t * state, vx_world_t* world){    
    // draw intended route
    vx_buffer_t *wrld = vx_world_get_buffer(world, "origin");
    float x0 = 0*15,y0 = 0*15,x1 = 0.6096*15,y1=0*15,x2 = 0.6096*15,y2 = -0.9144*15,x3 = 0*15,y3 = -0.9144*15;
    float origin_pts[] = {x0,y0,0,x1,y1,0,x1,y1,0,x2,y2,0,x2,y2,0,x3,y3,0,x3,y3,0,x0,y0,0};
    int origin_pts_size = 8;
    vx_resc_t *origin_verts = vx_resc_copyf(origin_pts, origin_pts_size*3);
    vx_buffer_add_back(wrld, vxo_lines(origin_verts, origin_pts_size, GL_LINES, vxo_lines_style(vx_orange, 2.0f)));
    //vx_buffer_swap(vx_world_get_buffer(world, "origin"));
    // make legend
    int lgnd_pts = 2;
    vx_object_t *lgnd = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> Legend\n");
    vx_object_t *odm = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> Odometry: \n");
    float odm_ln[6] = {20, 0, 0, 0, 0, 0};
    vx_resc_t *odm_verts = vx_resc_copyf(odm_ln, lgnd_pts*3);
    vx_object_t *imu = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> IMU: \n");
    float imu_ln[6] = {20, 0, 0, 0, 0, 0};
    vx_resc_t *imu_verts = vx_resc_copyf(imu_ln, lgnd_pts*3);
    vx_object_t *cmd = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> Command Path: \n");
    float cmd_ln[6] = {20, 0, 0, 0, 0, 0};
    vx_resc_t *cmd_verts = vx_resc_copyf(cmd_ln, lgnd_pts*3);
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(100, 180), vxo_mat_scale(0.8), lgnd)));
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(100, 165), vxo_mat_scale(0.6), odm)));
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(140, 165), vxo_mat_scale(1.0),
                vxo_lines(odm_verts, lgnd_pts, GL_LINES, vxo_points_style(vx_blue, 2.0f)))));
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(117, 150), vxo_mat_scale(0.6), imu)));
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(140, 150), vxo_mat_scale(1.0),
                vxo_lines(imu_verts, lgnd_pts, GL_LINES, vxo_points_style(vx_red, 2.0f)))));
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(88, 135), vxo_mat_scale(0.6), cmd)));
    vx_buffer_add_back(wrld, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(140, 135), vxo_mat_scale(1.0),
                vxo_lines(cmd_verts, lgnd_pts, GL_LINES, vxo_points_style(vx_orange, 2.0f)))));
    vx_buffer_swap(wrld);

}

void* render_loop(void*) {

	/*while (1) {
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
	}*/
    //state_t * state = (state_t*) data;
    while(state->running){
        vx_buffer_t *buf = vx_world_get_buffer(state->world,"pose_data");
        //for(int i = 0; i < state->pose_data.size()-1;++i){
        //    float pts[] = {state->pose_data[i].get_x_pos(),state->pose_data[i].get_y_pos(),0.0,
        //                    state->pose_data[i+1].get_x_pos(),state->pose_data[i].get_y_pos(),0.0};
            //float pts[] = {0*15,0*15,0,15,15,0};
            //vx_resc_t *verts = vx_resc_copyf(pts,6);
            //vx_buffer_add_back(buf,vxo_lines(verts,1,GL_LINES,vxo_lines_style(vx_red,2.0f)));
        //}
        char buffer[50];
        sprintf(buffer,"<<center, #000000>> pose_size: %d \n",state->pose_data.size());
        vx_object_t *data_size = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, buffer);
        vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(0, 0), vxo_mat_scale(0.6), data_size)));
        vx_buffer_swap(buf);
        usleep(5000);
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
	pthread_mutex_destroy(&state->run_mutex);
	//pthread_mutex_destroy(&state->odo_points_mutex);
	pthread_mutex_destroy(&state->scans_mutex);
	//pthread_mutex_destroy(&state->odo_curr_mutex);
	//lcm_destroy(state->lcm);
    pthread_mutex_lock(&(state->run_mutex));
    state->running = 0;
    pthread_mutex_unlock(&(state->run_mutex));
    getopt_destroy(state->gopt);
}

static state_t * state_create()
{
	state_t * state = (state_t * ) calloc(1, sizeof(state_t));
	pthread_mutex_lock(&(state->run_mutex));
    state->running = 1;
    pthread_mutex_unlock(&(state->run_mutex));
	state->app.impl=state;
	state->app.display_started=display_started;
	state->app.display_finished=display_finished;
	state->world = vx_world_create();
    state->layers = zhash_create(sizeof(vx_display_t*),sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
	pthread_mutex_init (&state->mutex, NULL);

	if (pthread_mutex_init(&state->run_mutex, NULL)) {
		printf("run mutex init failed\n");
		exit(1);
	}
	if (pthread_mutex_init(&state->pose_curr_mutex, NULL)) {
		printf("pose_curr mutex init failed\n");
		exit(1);
	}
	if (pthread_mutex_init(&state->pose_points_mutex, NULL)) {
		printf("pose_points_mutex init failed\n");
		exit(1);
	}
	if (pthread_mutex_init(&state->scans_mutex, NULL)) {
		printf("scans mutex init failed\n");
		exit(1);
	}

	//state->pose_y_curr = 0;
	//state->pose_x_curr = 0;
	//state->pose_heading_curr = 0;
	state->gopt = getopt_create();
	return state;
}

int main(int argc, char ** argv)
{
    fp_pose = fopen("pose_test.txt","w");
	vx_global_init(); // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library

	state = state_create();
	if (!getopt_parse(state->gopt, argc, argv, 1)) {
		printf("couldn't parse getopt\n");
		exit(1);
	}


	//pthread_t draw_thread_pid;
	//pthread_create(&draw_thread_pid, NULL, draw_thread, NULL);
    draw(state,state->world);
    pthread_create(&state->animate_thread,NULL,render_loop,NULL);

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
	gtk_main(); // Blocks as long as GTK window is open
	gdk_threads_leave();
	vx_gtk_display_source_destroy(appwrap);
    pthread_join(state->animate_thread,NULL);

    if(!state->lcm.good()){
        printf("LCM not good, exit\n");
        exit(1);
    }
	state->lcm.subscribe("MAEBOT_LASER_SCAN", &maebot_laser_scan_handler::handleMessage, &state->laser_scan_handler);

	state->lcm.subscribe("MAEBOT_POSE", &maebot_pose_handler::handleMessage, &state->pose_handler);

	pthread_t laser_thread_pid;
	pthread_create(&laser_thread_pid, NULL, laser_scan_thread, NULL);

	pthread_t pose_thread_pid;
	pthread_create(&pose_thread_pid, NULL, pose_thread, NULL);

    //pthread_t lcm_thread_pid;
    //pthread_create(&lcm_thread_pid,NULL,run_lcm,NULL);
    //while(state->lcm.handle() == 0);
	printf("All subscribed\n");

	state_destroy(state);
	vx_global_destroy();
}
