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

static const char* MAP_TO_READ;

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

        std::vector<maebot_pose_t> our_path;
        std::vector<maebot_pose_t> collins_path;
        maebot_pose_t curr_collin_pose;
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
            pose_fp = fopen("pose_data.txt","w");
            odo_fp = fopen("odo_data.txt","w");
            error_fp = fopen("error_data.txt","w");

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

            collins_path.push_back(*msg);
            curr_collin_pose = *msg;
            fprintf(pose_fp,"%lld %f %f\n",msg->utime,msg->x,msg->y);
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
                our_path.push_back(best);
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
                fprintf(odo_fp,"%lld %f %f\n",best.utime,best.x,best.y);
                fprintf(error_fp,"%lld %f\n",best.utime,error);
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
            render_grid(state);
            eecs467::OccupancyGrid& grid = state->map.get_grid();
            eecs467::Point<float> origin = grid.originInGlobalFrame();
            vx_object_t *vo = vxo_chain(vxo_mat_translate3(origin.x*15,origin.y*15,-0.01),
                    vxo_mat_scale((double)grid.metersPerCell()*15),
                    vxo_image_from_u8(state->image_buf,0,0));
            vx_buffer_add_back(buf,vo);
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
                /*render_grid(state);
                eecs467::OccupancyGrid& grid = state->map.get_grid();
                eecs467::Point<float> origin = grid.originInGlobalFrame();
                vx_object_t *vo = vxo_chain(vxo_mat_translate3(origin.x*15,origin.y*15,-0.01),
                        vxo_mat_scale((double)grid.metersPerCell()*15),
                        vxo_image_from_u8(state->image_buf,0,0));
                vx_buffer_add_back(buf,vo);
                */
                /*if(state->path.size() > 1){
                  for(int i = 1; i < state->path.size();++i){
                  float pts[] = {state->path[i].x*15,state->path[i].y*15,0.0,
                  state->path[i-1].x*15,state->path[i-1].y*15,0.0};
                //float pts[] = {0*15,0*15,0,15,15,0};
                vx_resc_t *verts = vx_resc_copyf(pts,6);
                vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_red,2.0f)));
                }*/
                /*for(int i=0;i<state->curr_lasers.size();i+=5){
                    float pts[] = {state->curr_lasers[i].get_x_pos()*15,state->curr_lasers[i].get_y_pos()*15,0.0,
                        state->curr_lasers[i].get_x_end_pos()*15,state->curr_lasers[i].get_y_end_pos()*15,0.0};
                    vx_resc_t *verts = vx_resc_copyf(pts,6);
                    vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_blue,1.0f)));
                }*/
                /*char buffer[50];
                  sprintf(buffer,"<<center, #000000>> (%.2f,%.2f,%.2f)\n",state->path.back().x,state->path.back().y,state->path.back().theta);
                  vx_object_t *data_size = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, buffer);
                  vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_LEFT, vxo_chain(vxo_mat_translate2(70,8),vxo_mat_scale(0.8),data_size)));
                  }
                  char buffer[50];
                  sprintf(buffer,"<<center, #000000>> laser_size: %d \n",state->curr_lasers.size());
                  vx_object_t *data_size = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, buffer);
                  vx_buffer_add_back(buf, vxo_pix_coords(VX_ORIGIN_BOTTOM_RIGHT, vxo_chain(vxo_mat_translate2(-70, 8), vxo_mat_scale(0.8), data_size)));
                  */
                if(state->particles.get_size() > 1){
                    float* pts = state->particles.get_particle_coords();
                    int npoints = state->particles.get_size();
                    vx_resc_t *verts = vx_resc_copyf(pts, npoints*3);
                    vx_buffer_add_back(buf, vxo_points(verts, npoints, vxo_points_style(vx_green, 2.0f)));
                } 
                if(state->our_path.size() > 1){
                    for(int i = 1; i < state->our_path.size();++i){
                        float pts[] = {state->our_path[i].x*15,state->our_path[i].y*15,0.0,
                            state->our_path[i-1].x*15,state->our_path[i-1].y*15,0.0};
                        //float pts[] = {0*15,0*15,0,15,15,0};
                        vx_resc_t *verts = vx_resc_copyf(pts,6);
                        vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_red,2.0f)));
                    }
                }
                if(state->collins_path.size() > 1){
                    for(int i = 1; i < state->collins_path.size();++i){
                        float pts[] = {state->collins_path[i].x*15,state->collins_path[i].y*15,0.0,
                            state->collins_path[i-1].x*15,state->collins_path[i-1].y*15,0.0};
                        //float pts[] = {0*15,0*15,0,15,15,0};
                        vx_resc_t *verts = vx_resc_copyf(pts,6);
                        vx_buffer_add_back(buf,vxo_lines(verts,2,GL_LINES,vxo_lines_style(vx_black,2.0f)));
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
    MAP_TO_READ = argv[1];
    printf("%s",MAP_TO_READ);
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
    gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
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
