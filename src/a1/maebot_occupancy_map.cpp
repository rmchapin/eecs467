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
            map = occupancy_map(5.0,5.0,0.05,1.0); //if s_rate changes here, correction in samples class needs adjustment
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
            //pthread_create(&compute_thread_pid,NULL,&state_t::compute_thread,this);
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
            //for(int i=0;i<msg->num_ranges;++i){
            //    maebot_laser l = maebot_laser(msg->times[i],msg->ranges[i],msg->thetas[i],msg->intensities[i],0,0);
            //    curr_lasers.push_back(l);
            //}
            //printf("%d\n",msg->num_ranges);
            matcher.process(); 
            curr_lasers = matcher.get_processed_laser();
            /*if(!(matcher.get_processed_laser(curr_lasers))){
                pthread_mutex_unlock(&data_mutex);
                return;
            }*/
            if(curr_lasers.empty()){
                pthread_mutex_unlock(&data_mutex);
                return;        
            }
            //printf("%d\n",state->curr_lasers.size());
            map.update(curr_lasers);
            pthread_mutex_unlock(&data_mutex);
        }

        static void* compute_thread(void *input){
            state_t* state = (state_t*) input;
            while(1){
                usleep(1000);
                pthread_mutex_lock(&state->data_mutex);
                state->matcher.process(); 
                //state->curr_lasers = state->matcher.get_processed_laser();
                if(!(state->matcher.get_processed_laser(state->curr_lasers))){
                    pthread_mutex_unlock(&state->data_mutex);
                    continue;
                }
                //printf("%d\n",state->curr_lasers.size());
                state->map.update(state->curr_lasers);
                pthread_mutex_unlock(&state->data_mutex);
            }
        }

        static void* run_lcm(void *input){
            //pthread_mutex_lock(&(state->run_mutex));
            state_t* state = (state_t*) input;
            while(1){
                //pthread_mutex_lock(&(state->data_mutex));
                state->lcm.handle();
                //pthread_mutex_unlock(&(state->data_mutex));
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
                    state->image_buf->buf[(y * state->image_buf->stride) + x] = to_grayscale(grid.logOdds(y,x));
                }
            }
        }

        static void save_map(state_t *state){
            FILE *fp;
            fp = fopen("occupancy_map.txt","w");
            eecs467::OccupancyGrid& grid = state->map.get_grid();
            fprintf(fp,"%d\n",grid.heightInCells());
            fprintf(fp,"%d\n",grid.widthInCells());
            for(size_t y = 0; y < grid.heightInCells();y++){
                for(size_t x = 0; x < grid.widthInCells(); x++){
                    fprintf(fp,"%d\n",grid.logOdds(y,x));
                }
            }
            fclose(fp);
        }

        static void* render_loop(void* data) {

            state_t * state = (state_t*) data;

            while (1) {
                pthread_mutex_lock(&state->data_mutex);
                vx_buffer_t *buf = vx_world_get_buffer(state->world,"pose_data");
                render_grid(state);
                eecs467::OccupancyGrid& grid = state->map.get_grid();
                eecs467::Point<float> origin = grid.originInGlobalFrame();
                vx_object_t *vo = vxo_chain(vxo_mat_translate3(origin.x*15,origin.y*15,-0.01),
                        vxo_mat_scale((double)grid.metersPerCell()*15),
                        vxo_image_from_u8(state->image_buf,0,0));
                vx_buffer_add_back(buf,vo);

                if(state->path.size() > 1){
                    for(int i = 1; i < state->path.size();++i){
                        float pts[] = {state->path[i].x*15,state->path[i].y*15,0.0,
                            state->path[i-1].x*15,state->path[i-1].y*15,0.0};
                        //float pts[] = {0*15,0*15,0,15,15,0};
                        vx_resc_t *verts = vx_resc_copyf(pts,6);
                        vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_red,2.0f)));
                    }
                    for(int i=0;i<state->curr_lasers.size();i+=5){
                        float pts[] = {state->curr_lasers[i].get_x_pos()*15,state->curr_lasers[i].get_y_pos()*15,0.0,
                            state->curr_lasers[i].get_x_end_pos()*15,state->curr_lasers[i].get_y_end_pos()*15,0.0};
                        vx_resc_t *verts = vx_resc_copyf(pts,6);
                        vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_blue,1.0f)));
                    }
                    char buffer[50];
                    sprintf(buffer,"<<center, #000000>> (%.2f,%.2f,%.2f)\n",state->path.back().x,state->path.back().y,state->path.back().theta);
                    vx_object_t *data_size = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, buffer);
                    vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vxo_chain(vxo_mat_translate2(70,8),vxo_mat_scale(0.8),data_size)));
                }
                char buffer[50];
                sprintf(buffer,"<<center, #000000>> laser_size: %d \n",state->curr_lasers.size());
                vx_object_t *data_size = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, buffer);
                vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_RIGHT, vxo_chain(vxo_mat_translate2(-70, 8), vxo_mat_scale(0.8), data_size)));

                pthread_mutex_unlock(&state->data_mutex);
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

    state.init_thread();
    //while(1){
    //    state.lcm.handle();
    //}
    //vx_global_init(); 
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

    //while(1){}
    vx_global_destroy();
}
