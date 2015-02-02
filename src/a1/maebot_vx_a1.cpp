#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <vector>
#include <deque>

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

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include <math/point.hpp>
#define GRID_WIDTH_M 10
#define GRID_HEIGHT_M 10
#define CELL_WIDTH_M 0.05
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

class state_t
{
public:
    lcm::LCM lcm;
    pthread_mutex_t run_mutex;
	getopt_t* gopt;

	//pose
	int pose_index = 0;
    std::deque<maebot_pose_data> pose_data;
	pthread_mutex_t pose_mutex;

	// lidar
    maebot_laser_data lasers;
    laser_array other_lasers;
	pthread_mutex_t scans_mutex;

    // vx stuff	
	int running;
	vx_application_t app;
	vx_world_t * world;
	zhash_t * layers;
	pthread_mutex_t mutex; // for accessing the arrays
	pthread_t animate_thread;

	eecs467::OccupancyGrid grid;
	image_u8_t *image_buf;
};

maebot_pose_data interpolate(int64_t time, state_t * state){

    //find pose pointers that lazer ray lies between
    maebot_pose_data data1;
    maebot_pose_data data2;

    for(std::deque<maebot_pose_data>::iterator it = state->pose_data.end() -1; 
        it >= state->pose_data.begin(); --it){

        if(it->get_timestamp() < time){
            data2 = (*it);
            data1 = *(++it);
            break;
        }

    }

    //data 2 is old, data 1 is most recent
    //interpolation
    double diff_x = data1.get_x_pos() - data2.get_x_pos();
    double diff_y = data1.get_y_pos() - data2.get_y_pos();
    double diff_time = data1.get_timestamp() - data2.get_timestamp();
    double diff_theta = data1.get_theta() - data2.get_theta();

    double fraction = (time-data2.get_timestamp())/ diff_time;
    double calc_x = (diff_x*fraction)+ data2.get_x_pos();
    double calc_y = (diff_y*fraction)+ data2.get_y_pos();
    double calc_theta = (diff_theta*fraction)+ data2.get_theta();

    //creates return variable
    maebot_pose_data temp = maebot_pose_data(time,
            calc_x,
            calc_y,
            calc_theta);

    return temp;

}

void update_grid(maebot_pose_data pose, float laser_th, float range, eecs467::OccupancyGrid grid)
{
	int64_t mae_t = pose.get_timestamp();
	double mae_x = pose.get_x_pos();
	double mae_y = pose.get_y_pos();
	double mae_th = pose.get_theta();

	double cos_th = cos(mae_th - laser_th);
	double sin_th = cos(mae_th - laser_th);

	for (double sample_m = SAMPLE_SPACING; sample_m < range; sample_m += SAMPLE_SPACING)
	{
		double sample_x = mae_x + (sample_m*cos_th);
		double sample_y = mae_y + (sample_m*sin_th);
			
		eecs467::Point<int> sample_cell = global_position_to_grid_cell(eecs467::Point<double>(sample_x, sample_y), grid);
		if (grid.isCellInGrid(sample_cell.x, sample_cell.y))
        {
    		int8_t odds = grid.logOdds(sample_cell.x, sample_cell.y);
    		odds -= FREE_ADJUST_FACTOR;
    		grid.setLogOdds(sample_cell.x, sample_cell.y, odds);
        }
	}

	//positive adjust for ray termination cell
	double laser_term_x = mae_x + range*cos_th;
	double laser_term_y = mae_y + range*sin_th;
	eecs467::Point<int> laser_term_cell = global_position_to_grid_cell(eecs467::Point<double>(laser_term_x, laser_term_y), grid);
	
    if (grid.isCellInGrid(laser_term_cell.x, laser_term_cell.y))
    {
        int8_t odds = grid.logOdds(laser_term_cell.x, laser_term_cell.y);
    	odds += OCCUPIED_ADJUST_FACTOR;
    	grid.setLogOdds(laser_term_cell.x, laser_term_cell.y, odds);
    }
}

void* compute_thread(void *input)
{
	state_t* state = (state_t*) input;

	while (1)
	{
		while (state->other_lasers.size() > 1)
		{
			if (state->pose_data[state->pose_index].get_timestamp() > state->other_lasers.front_time())
			{
				maebot_pose_data calc_pose = interpolate(state->other_lasers.front_time(), state);
				update_grid(calc_pose, state->other_lasers.front_theta(), state->other_lasers.front_range(), state->grid);
				state->other_lasers.pop();
			}
		}
	}
}

void* run_lcm(void *input){
    //pthread_mutex_lock(&(state->run_mutex));
    state_t* state = (state_t*) input;
    while(1){
        //pthread_mutex_unlock(&(state->run_mutex));
        state->lcm.handle();
        //pthread_mutex_lock(&(state->run_mutex));
    }
    //pthread_mutex_unlock(&(state->run_mutex));
    return NULL;
}

static void pose_data_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_pose_t *msg, state_t* state){
    int res = system ("clear");

    //create pose object
    maebot_pose_data new_pose = maebot_pose_data(msg->utime,
									             msg->x,
									             msg->y,
									             msg->theta);

    //lock mutex and push into state
    pthread_mutex_lock(&state->pose_mutex);
    state->pose_data.push_back(new_pose);
    state->pose_index++;
	pthread_mutex_unlock(&state->pose_mutex);
}

static void laser_scan_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_laser_scan_t *msg, state_t *state){
    int res = system("clear");
    
    pthread_mutex_lock(&state->scans_mutex);
    state->lasers = maebot_laser_data(msg->utime,
           msg->num_ranges,
           msg->ranges,
           msg->thetas,
           msg->times,
           msg->intensities,
           state->pose_data[state->pose_index]);
    //printf("%ld\n",msg->num_ranges);

	for (int i = 0; i < msg->num_ranges; i++)
	{
	    state->other_lasers.add_ray(msg->times[i], msg->thetas[i], msg->ranges[i]);
	}

	pthread_mutex_unlock(&state->scans_mutex);
}

static void draw(state_t* state, vx_world_t* world){    
    // draw intended route
    vx_buffer_t *wrld = vx_world_get_buffer(world, "origin");
    //float x0 = 0*15,y0 = 0*15,x1 = 0.6096*15,y1=0*15,x2 = 0.6096*15,y2 = -0.9144*15,x3 = 0*15,y3 = -0.9144*15;
    //float origin_pts[] = {x0,y0,0,x1,y1,0,x1,y1,0,x2,y2,0,x2,y2,0,x3,y3,0,x3,y3,0,x0,y0,0};
    //int origin_pts_size = 8;
    //vx_resc_t *origin_verts = vx_resc_copyf(origin_pts, origin_pts_size*3);
    //vx_buffer_add_back(wrld, vxo_lines(origin_verts, origin_pts_size, GL_LINES, vxo_lines_style(vx_orange, 2.0f)));
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

static uint8_t to_grayscale(uint8_t logOdds)
{
	return 127 - logOdds;
}

static void render_grid(state_t * state)
{
	for (size_t y = 0; y < state->grid.heightInCells(); y++)
	{
		for (size_t x = 0; x < state->grid.widthInCells(); x++)
		{
			state->image_buf->buf[(y * state->image_buf->stride) + x] = to_grayscale(state->grid.logOdds(x,y));
		}
	}

}

void* render_loop(void* data) {
	
    state_t * state = (state_t*) data;

    while (1) {
        vx_buffer_t *buf = vx_world_get_buffer(state->world,"pose_data");
        if(state->pose_data.size() > 1){
            for(int i = 1; i < state->pose_data.size();++i){
                float pts[] = {state->pose_data[i].get_x_pos()*15,state->pose_data[i].get_y_pos()*15,0.0,
                                state->pose_data[i-1].get_x_pos()*15,state->pose_data[i-1].get_y_pos()*15,0.0};
                //float pts[] = {0*15,0*15,0,15,15,0};
                vx_resc_t *verts = vx_resc_copyf(pts,6);
                vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_red,2.0f)));
            }
            std::vector<eecs467::Point<float>> end = state->lasers.get_end_points();
            maebot_pose_data p = state->lasers.get_curr_pose();
            for(int i=0;i<state->lasers.get_num_ranges();i+=10){
                float pts[] = {p.get_x_pos()*15,p.get_y_pos()*15,0.0,
                               end[i].x*15,end[i].y*15,0.0};
                vx_resc_t *verts = vx_resc_copyf(pts,6);
                vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_blue,1.0f)));
            }
        }

        render_grid(state);
        //vx_resc_t *vr = resc_copyui(state->image_buf,state->image_buf->stride*state->image_buf->height);
        vx_object_t *vo = vxo_image_from_u8(state->image_buf,NULL,NULL);
        vx_buffer_add_back(buf,vo);

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

	/*if (state->img != NULL)
		image_u32_destroy(state->img);*/

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

    image_u8_destroy(state->image_buf);

    pthread_mutex_unlock(&(state->run_mutex));
    getopt_destroy(state->gopt);
}

static state_t * state_create()
{
	//state_t * state = (state_t * ) calloc(1, sizeof(state_t));
    state_t *state = new state_t;
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
	if (pthread_mutex_init(&state->pose_mutex, NULL)) {
		printf("pose_curr mutex init failed\n");
		exit(1);
	}
	if (pthread_mutex_init(&state->scans_mutex, NULL)) {
		printf("scans mutex init failed\n");
		exit(1);
	}

	state->grid = eecs467::OccupancyGrid(GRID_WIDTH_M, GRID_HEIGHT_M, CELL_WIDTH_M);
	state->image_buf = image_u8_create(state->grid.widthInCells(), state->grid.heightInCells());

	state->gopt = getopt_create();
	return state;
}

int main(int argc, char ** argv)
{
	vx_global_init(); // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library

	state_t* state = state_create();
	if (!getopt_parse(state->gopt, argc, argv, 1)) {
		printf("couldn't parse getopt\n");
		exit(1);
	}
    
    if(!state->lcm.good()){
        printf("LCM not good, exit\n");
        exit(1);
    }
	//state->lcm.subscribe("MAEBOT_LASER_SCAN", &maebot_laser_scan_handler::handleMessage, &state->laser_scan_handler);

	//state->lcm.subscribe("MAEBOT_POSE", &maebot_pose_handler::handleMessage, &state->pose_handler);

    state->lcm.subscribeFunction("MAEBOT_POSE",pose_data_handler,state);
    state->lcm.subscribeFunction("MAEBOT_LASER_SCAN", laser_scan_handler,state);
	//pthread_t laser_thread_pid;
	//pthread_create(&laser_thread_pid, NULL, laser_scan_thread, NULL);

	//pthread_t pose_thread_pid;
	//pthread_create(&pose_thread_pid, NULL, pose_thread, NULL);

    pthread_t lcm_thread_pid;
    pthread_create(&lcm_thread_pid,NULL,run_lcm,state);
    //while(state->lcm.handle() == 0);
	printf("All subscribed\n");

	pthread_t compute_thread_pid;
    pthread_create(&compute_thread_pid,NULL,compute_thread,state);

	//pthread_t draw_thread_pid;
	//pthread_create(&draw_thread_pid, NULL, draw_thread, NULL);
    draw(state,state->world);
    pthread_create(&state->animate_thread,NULL,render_loop,state);

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
    
       //while(1);
	state_destroy(state);
    //delete state;
	vx_global_destroy();
}
