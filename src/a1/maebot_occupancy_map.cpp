#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <vector>
#include <deque>
#include <iostream>

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
//#include "maebot_data.hpp"
//#define BASE_LENGTH 0.08 // meters
//#define METERS_PER_TICK 0.00020944 // meters
//#define ADC_PER_METER_PER_SECOND_PER_SECOND 1670.13251784
//define ADC_PER_RADIANS_PER_SECOND 7505.74711621

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include <math/point.hpp>
#include "occupancy_map.hpp"
#include "laser_matcher.hpp"

class state_t
{
    public:
        occupancy_map map;

        lcm::LCM lcm;
        pthread_t lcm_thread_pid;

        pthread_mutex_t data_mutex;
        pthread_mutex_t run_mutex;
        std::deque<maebot_pose_t> path;
        std::deque<maebot_laser> curr_lasers;
        laser_matcher matcher;
        pthread_t compute_thread_pid;

        // vx stuff	
        int running;
        vx_application_t app;
        vx_world_t * world;
        zhash_t * layers;
        vx_gtk_display_source_t* appwrap;
        pthread_mutex_t mutex; // for accessing the arrays
        pthread_t animate_thread;
        image_u8_t *image_buf;
    public:
        state_t(){
            layers = zhash_create(sizeof(vx_display_t*),sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
            map = occupancy_map(5.0,5.0,0.05,1.0);
            running = 1;
            app.impl= this;
            app.display_started = display_started;
            app.display_finished = display_finished;
            world = vx_world_create();
            pthread_mutex_init (&mutex, NULL);

            if (pthread_mutex_init(&run_mutex, NULL)) {
                printf("run mutex init failed\n");
                exit(1);
            }
            if (pthread_mutex_init(&data_mutex, NULL)) {
                printf("pose_curr mutex init failed\n");
                exit(1);
            }
            image_buf = nullptr; 
            lcm.subscribe("MAEBOT_POSE",&state_t::pose_data_handler,this);
            lcm.subscribe("MAEBOT_LASER_SCAN", &state_t::laser_scan_handler,this);
        }

        ~state_t(){
            vx_world_destroy(world);
            assert(zhash_size(layers) == 0);
            zhash_destroy(layers);
            pthread_mutex_destroy(&mutex);
            pthread_mutex_destroy(&run_mutex);
            //pthread_mutex_destroy(&state->odo_points_mutex);
            pthread_mutex_destroy(&data_mutex);
            //pthread_mutex_destroy(&state->odo_curr_mutex);
            //lcm_destroy(state->lcm);
            running = 0;
            image_u8_destroy(image_buf);
        }

        void init_thread(){
            pthread_create(&lcm_thread_pid,NULL,&state_t::run_lcm,this);
            pthread_create(&animate_thread,NULL,&state_t::render_loop,this);
            pthread_create(&compute_thread_pid,NULL,&state_t::compute_thread,this);
        }


        void pose_data_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_pose_t *msg){
            pthread_mutex_lock(&data_mutex);
            matcher.push_pose(msg);
            path.push_back(*msg);
            pthread_mutex_unlock(&data_mutex);
        }

        void laser_scan_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_laser_scan_t *msg){
            pthread_mutex_lock(&data_mutex);
            matcher.push_laser(msg);
            pthread_mutex_unlock(&data_mutex);
        }

        static void* compute_thread(void *input){
            state_t* state = (state_t*) input;
            while(1){
                usleep(1000);
                pthread_mutex_lock(&state->data_mutex);
                state->matcher.process();
                //std::deque<maebot_laser> lasers;
                if(!state->matcher.get_processed_laser(state->curr_lasers)){
                    pthread_mutex_unlock(&state->data_mutex);
                    continue;
                }
                state->map.update(state->curr_lasers);
                pthread_mutex_unlock(&state->data_mutex);
            }
        }

        static void* run_lcm(void *input){
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
        static void draw(state_t* state, vx_world_t* world){    

        }

        static uint8_t to_grayscale(int8_t logOdds)
        {
            return 127 - logOdds;
        }

        static void render_grid(state_t * state)
        {
            eecs467::OccupancyGrid& grid = state->map.get_grid();
            if(state->image_buf == nullptr){
                state->image_buf = image_u8_create(grid.widthInCells(),grid.heightInCells());
            }
            for (size_t y = 0; y < grid.heightInCells(); y++)
            {
                for (size_t x = 0; x < grid.widthInCells(); x++)
                {
                    state->image_buf->buf[(y * state->image_buf->stride) + x] = to_grayscale(grid.logOdds(x,y));
                }
            }
        }

        static void* render_loop(void* data) {

            state_t * state = (state_t*) data;

            while (1) {
                vx_buffer_t *buf = vx_world_get_buffer(state->world,"pose_data");
                render_grid(state);
                vx_object_t *vo = vxo_image_from_u8(state->image_buf,0,0);
                vx_buffer_add_back(buf,vo);
                /*if(state->pose_data.size() > 1){
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
                                */
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


};

state_t state;

int main(int argc, char ** argv)
{
    // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library

    vx_global_init(); 
    //pthread_t lcm_thread_pid;
    //pthread_create(&lcm_thread_pid,NULL,run_lcm,state);
    //pthread_t compute_thread_pid;
    //pthread_create(&compute_thread_pid,NULL,compute_thread,state);

    //pthread_t draw_thread_pid;
    //pthread_create(&draw_thread_pid, NULL, draw_thread, NULL);
    //draw(state,state->world);
    //pthread_create(&state->animate_thread,NULL,render_loop,state);
    state.init_thread();
    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);

    state.appwrap = vx_gtk_display_source_create(&state.app);
    GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas = vx_gtk_display_source_get_widget(state.appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show (window);
    gtk_widget_show (canvas); // XXX Show all causes errors!
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);
    gtk_main(); // Blocks as long as GTK window is open
    gdk_threads_leave();
    vx_gtk_display_source_destroy(state.appwrap);
    pthread_join(state.animate_thread,NULL);

    vx_global_destroy();
}
