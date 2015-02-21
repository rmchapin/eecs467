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
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "particle_data.hpp"
#include "action_model.hpp"
#include "pose_tracker.hpp"

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include <math/point.hpp>
#include "occupancy_map.hpp"
#include <math/gsl_util_rand.h>

static const char* MAP_TO_READ = "figure_eight.txt";

class state_t
{
    public:
        occupancy_map map;

        lcm::LCM lcm;
        pthread_t lcm_thread_pid;

        pthread_mutex_t data_mutex;
        pthread_mutex_t run_mutex;

        particle_data particles;
        //action_model action_error_model;
        //pose_tracker bot_tracker;

        //std::vector<maebot_pose_t> our_path;
        //std::vector<maebot_pose_t> collins_path;
        //std::vector<maebot_pose_t> our_display_path;
        //std::vector<maebot_pose_t> collins_display_path;
        maebot_pose_t curr_collin_pose;
        std::vector<float> error_path;
        //std::deque<maebot_laser> curr_lasers;
        //bool first_scan;
        // vx stuff	
        vx_application_t app;
        vx_world_t * world;
        zhash_t * layers;
        vx_gtk_display_source_t* appwrap;
        pthread_mutex_t mutex; // for accessing the arrays
        pthread_t animate_thread;
        image_u8_t *image_buf;
        FILE *pose_fp;
        FILE *odo_fp;
        FILE *error_fp;

    public:
        state_t()
        {
            //initialize particles at (0,0,0)
            maebot_pose_t temp;
            temp.x=0;
            temp.y=0;
            temp.theta=0;

            //read_map();
            map = occupancy_map(5.0,5.0,0.05,1.0);
            particles = particle_data(1000, temp, &map.grid);
            read_map();
            //first_scan = true;
            //GUI init stuff
            layers = zhash_create(sizeof(vx_display_t*),sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
            app.impl= this;
            app.display_started = display_started;
            app.display_finished = display_finished;
            world = vx_world_create();
            pthread_mutex_init (&mutex, NULL);
            gslu_rand_seed(); 
            //action_error_model = action_model();
            //bot_tracker = pose_tracker();

            if (pthread_mutex_init(&run_mutex, NULL)) {
                printf("run mutex init failed\n");
                exit(1);
            }
            if (pthread_mutex_init(&data_mutex, NULL)) {
                printf("pose_curr mutex init failed\n");
                exit(1);
            }

            image_buf = nullptr;

            lcm.subscribe("MAEBOT_POSE", &state_t::pose_handler, this);
            lcm.subscribe("MAEBOT_MOTOR_FEEDBACK", &state_t::odo_handler, this);
            lcm.subscribe("MAEBOT_LASER_SCAN", &state_t::laser_scan_handler, this);
            //pose_fp = fopen("pose_data.txt","w");
            //odo_fp = fopen("odo_data.txt","w");
            //error_fp = fopen("error_data.txt","w");
        }

        ~state_t()
        {
            vx_world_destroy(world);
            assert(zhash_size(layers) == 0);
            zhash_destroy(layers);
            pthread_mutex_destroy(&mutex);
            pthread_mutex_destroy(&run_mutex);
            pthread_mutex_destroy(&data_mutex);
            image_u8_destroy(image_buf);
        }

        void init_thread()
        {
            pthread_create(&lcm_thread_pid,NULL,&state_t::run_lcm,this);
            pthread_create(&animate_thread,NULL,&state_t::render_loop,this);
        }

        void pose_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_pose_t *msg){
            pthread_mutex_lock(&data_mutex);

            //collins_path.push_back(*msg);
            curr_collin_pose = *msg;
            //fprintf(pose_fp,"%lld %f %f\n",msg->utime,msg->x,msg->y);
            pthread_mutex_unlock(&data_mutex);
        }

        void odo_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_motor_feedback_t *msg)
        {
            //printf("odo handle\n");
            pthread_mutex_lock(&data_mutex);
            //printf("start pushing\n");
            //bot_tracker.push_msg(*msg, action_error_model);
            particles.push_odo(*msg);
            pthread_mutex_unlock(&data_mutex);

            pthread_mutex_lock(&data_mutex);
            if(particles.ready()){
                //printf("particle filter process\n");
                particles.update();
                //printf("best particle: %f %f\n",particles.get_best().x,particles.get_best().y);
                maebot_pose_t best = particles.get_best();
                //fprintf(odo_fp,"%lld %f %f\n",best.utime,best.x,best.y);
                float coeff = 0;
                if(best.y > curr_collin_pose.y){
                    coeff = 1;
                }
                else{
                    coeff = -1;
                }
                float dy = best.y - curr_collin_pose.y;
                float dx = best.x - curr_collin_pose.x;
                float error = coeff*sqrt(dx*dx+dy*dy);
                //fprintf(error_fp,"%lld %f\n",best.utime,error);
                //printf("%f\n",error);
                error_path.push_back(error);
                //our_path.push_back(best);
                //matcher.push_pose(&best);
            } 
            pthread_mutex_unlock(&data_mutex);
        }

        void laser_scan_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_laser_scan_t *msg)
        {
            pthread_mutex_lock(&data_mutex);
            //printf("laser handle\n");
            if(particles.processing == false){
                particles.push_scan(*msg);
                //matcher.push_laser(msg);
                //matcher.process();
                //curr_lasers = matcher.get_processed_laser();
                //if(curr_lasers.empty()){
                //    pthread_mutex_unlock(&data_mutex);
                //    return;
                //}
                //particles.s_model.update_grid(&map.grid);
            } 
            pthread_mutex_unlock(&data_mutex);
        }

        static void* run_lcm(void *input)
        {
            state_t* state = (state_t*) input;
            while(1){
                state->lcm.handle();
            }
            return NULL;
        }

        static void draw(state_t* state, vx_world_t* world)
        {    
            vx_buffer_t *buf = vx_world_get_buffer(state->world,"map");
            float pts[] = {-60,40,0,
                           -60,-40,0};
            vx_resc_t *verts = vx_resc_copyf(pts,6);
            vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_black,4.0f)));
            vx_object_t *neg_20 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> -2m\n");
            vx_object_t *pos_20 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> +2m\n");
            vx_object_t *zero = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> 0m\n");
            vx_object_t *neg_10 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> -1m\n");
            vx_object_t *pos_10 = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "<<center, #000000>> +1m\n");
            vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(-285, -170), vxo_mat_scale(0.8), neg_20))); 
            vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(-285, -85), vxo_mat_scale(0.8), neg_10))); 
            vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(-285, 0), vxo_mat_scale(0.8), zero))); 
            vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(-285, 85), vxo_mat_scale(0.8), pos_10))); 
            vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_CENTER, vxo_chain(vxo_mat_translate2(-285, 170), vxo_mat_scale(0.8), pos_20))); 
            vx_buffer_swap(buf);
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

        static void save_map(state_t *state)
        {
            FILE *fp;
            fp = fopen("empty_map.txt","w");
            eecs467::OccupancyGrid& grid = state->map.get_grid();
            fprintf(fp,"%d\n",grid.heightInCells());
            fprintf(fp,"%d\n",grid.widthInCells());
            for(size_t y = 0; y < grid.heightInCells();y++){
                for(size_t x = 0; x < grid.widthInCells(); x++){
                    fprintf(fp,"%d\n",grid.logOdds(x,y));
                }
            }
            fclose(fp);
        }

        void read_map()
        {
            FILE *fp;
            uint8_t temp;
            fp = fopen(MAP_TO_READ,"r");
            fscanf(fp,"%d\n",&temp);
            if(temp != map.grid.heightInCells()){
                std::cout << "Height not match\n";
                exit(1);
            }
            fscanf(fp,"%d\n",&temp);
            if(temp != map.grid.widthInCells()){
                std::cout << "Width not match\n";
                exit(1);
            }
            map = occupancy_map(5.0,5.0,0.05,1.0); //if s_rate changes here, correction in samples class needs adjustment
            for(size_t y = 0; y < map.grid.heightInCells();y++){
                for(size_t x = 0; x < map.grid.widthInCells(); x++){
                    fscanf(fp,"%d ",&temp);
                    map.grid.setLogOdds(x,y,temp);
                }
            }
            fclose(fp);
        }

        static void* render_loop(void* data)
        {
            state_t * state = (state_t*) data;

            while (1) {
                pthread_mutex_lock(&state->data_mutex);
                vx_buffer_t *buf = vx_world_get_buffer(state->world,"pose_data");
                if(state->error_path.size()>1){
                    for(int i=1;i<state->error_path.size();++i){
                        float pts[] = {(i-1)*0.3-60,state->error_path[i-1]*20,0,
                            		i*0.3-60   ,state->error_path[i]*20,0};
                        vx_resc_t *verts = vx_resc_copyf(pts,6);
                        vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_orange,2.0f)));
                    }
                }               
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

int main(int argc, char ** argv)
{
    state_t state;
    state.init_thread();
    //comment below disable the vx
    state.draw(&state,state.world);
    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);

    state.appwrap = vx_gtk_display_source_create(&state.app);
    GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas = vx_gtk_display_source_get_widget(state.appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window), 600, 400);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show (window);
    gtk_widget_show (canvas); // XXX Show all causes errors!
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);
    gtk_main(); // Blocks as long as GTK window is open

    gdk_threads_leave();
    vx_gtk_display_source_destroy(state.appwrap);
    //comment above disable vx
    pthread_join(state.animate_thread,NULL);

    vx_global_destroy();
}
