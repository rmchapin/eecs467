#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"

//lcm
#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

#define BAR_WIDTH_MAX 150
#define BAR_HEIGHT 20
#define SPACE_BETWEEN_BARS 40
#define SPACE_BETWEEN_BAR_AND_TEXT_CENTER 10
#define Y_OFFSET_BAR_TOP -40
#define Y_OFFSET_TITLE -20
#define NUM_SERVOS 6
double servo_angle[] 	 = { 0, 0, 0, 0, 0, 0 };
double servo_angle_min[] = {-3.14,-2.094,-2.094,-2.094,-2.618,-0.9 };
double servo_angle_max[] = { 3.14, 2.094, 2.094, 2.094, 2.618, 2.3 };

typedef struct
{
    int running;

    image_u32_t *img;

    vx_application_t app;

    vx_world_t * world;
    zhash_t * layers;

    pthread_mutex_t mutex; // for accessing the arrays
    pthread_t animate_thread;
    pthread_t status_thread;

    // LCM
    lcm_t *lcm;
    const char *status_channel;
} state_t;

//return val such that it is no smallr than min and no larger than max (if max < min, they are swapped).
static double clamp(double val, double min, double max)
{
	if(max < min)
	{
		double _temp = max;
		max = min;
		min = _temp;
	}
	if(val > max) return max;
	if(val < min) return min;
	return val;
}

static void
status_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const dynamixel_status_list_t *msg,
                void *user)
{
    // Print out servo positions
    /*for (int id = 0; id < msg->len; id++) {
        dynamixel_status_t stat = msg->statuses[id];
        printf ("[id %d]=%6.3f ",id, stat.position_radians);
    }
    printf ("\n");*/
	//set servo_angle[]'s values here.
	for(int i = 0; i < NUM_SERVOS; i++)
	{
		servo_angle[i] = (double) msg->statuses[i].position_radians;

		//Set servo angle value between bounds.
		servo_angle[i] = clamp(servo_angle[i], servo_angle_min[i], servo_angle_max[i]);
	}
}

void *
status_loop (void *data)
{
    state_t *state = data;
    dynamixel_status_list_t_subscribe (state->lcm,
                                       state->status_channel,
                                       status_handler,
                                       state);
    const int hz = 15;
    while (1) {
        // Set up the LCM file descriptor for waiting. This lets us monitor it
        // until something is "ready" to happen. In this case, we are ready to
        // receive a message.
        int status = lcm_handle_timeout (state->lcm, 1000/hz);
        if (status <= 0)
           continue;
	
        // LCM has events ready to be processed
    }

    return NULL;
}

static void draw(state_t * state, vx_world_t * world)
{
}

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;
    pthread_mutex_lock(&state->mutex);

    vx_layer_t * layer = NULL;

    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_remove(state->layers, &disp, NULL, &layer);

    vx_layer_destroy(layer);

    pthread_mutex_unlock(&state->mutex);
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
    state_t * state = app->impl;

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

}

static state_t * state_create()
{
    state_t * state = calloc(1, sizeof(state_t));
    state->running = 1;
    state->app.impl=state;
    state->app.display_started=display_started;
    state->app.display_finished=display_finished;


    state->world = vx_world_create();
    state->layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

    pthread_mutex_init (&state->mutex, NULL);

    return state;
}


void * render_loop(void * data)
{

    state_t * state = data;
    while(state->running) {

	vx_buffer_t *vb = vx_world_get_buffer(state->world, "bars");
	vx_object_t *vt_ = vxo_text_create(VXO_TEXT_ANCHOR_CENTER, "REXARM");
	vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP,
				vxo_chain(
					vxo_mat_translate3(0,Y_OFFSET_TITLE,0),
					vt_)));

	for(int i = 0; i < NUM_SERVOS; i++)
	{
    	float bar_back_col[] = {1, 0, 0, 0.05};
    	float bar_front_col[] = {1, 0, 0, 1};
	double v = servo_angle[i];
	if(v < 0)
		v = -v/servo_angle_min[i];
	else
		v = v/servo_angle_max[i];

	double w = v * BAR_WIDTH_MAX;
	double h = BAR_HEIGHT;
	double x = w/2;
	double y = Y_OFFSET_BAR_TOP-BAR_HEIGHT/2-i*(SPACE_BETWEEN_BARS+BAR_HEIGHT);

	bar_front_col[0] = v < 0 ? 1-v*v*v*v : 0;
	bar_front_col[1] = v > 0 ? 1-v*v*v*v : 0;

	vx_buffer_add_back(vb,
		           vxo_pix_coords(VX_ORIGIN_TOP, 
				vxo_chain(
				     vxo_mat_translate3(0,y,0),
		                     vxo_mat_scale3(BAR_WIDTH_MAX*2,h,1),
		                     vxo_box(vxo_mesh_style(bar_back_col)))));
	vx_buffer_add_back(vb,
		           vxo_pix_coords(VX_ORIGIN_TOP, 
				vxo_chain(
				     vxo_mat_translate3(x,y,0),
		                     vxo_mat_scale3(w,h,1),
		                     vxo_box(vxo_mesh_style(bar_front_col)))));
	char s[100];
	sprintf(s, "<<center,#0000ff>>[%d]: %f", i, servo_angle[i]);
	vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_TOP, s);
	vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP,
				vxo_chain(
					vxo_mat_translate3(0,y-SPACE_BETWEEN_BAR_AND_TEXT_CENTER,0),
					vt)));
	}
	vx_buffer_swap(vb);
        usleep(5000);
    }

    return NULL;
}

int main(int argc, char ** argv)
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool   (gopt, 'h', "help", 0, "Show help");
    getopt_add_bool (gopt, '\0', "no-gtk", 0, "Don't show gtk window, only advertise remote connection");
    getopt_add_string (gopt, '\0', "pnm", "", "Path for pnm file to render as texture (.e.g BlockM.pnm)");
    getopt_add_bool (gopt, '\0', "stay-open", 0, "Stay open after gtk exits to continue handling remote connections");
    getopt_add_string (gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");

    // parse and print help
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        printf ("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage (gopt);
        exit (1);
    }

    vx_global_init(); // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library

    state_t * state = state_create();

    //subscribe to lcm status channel
    state->lcm = lcm_create (NULL);
    state->status_channel = getopt_get_string (gopt, "status-channel");

    // Load a pnm from file, and repack the data so that it's understandable by vx
    if (strcmp(getopt_get_string(gopt,"pnm"),"")) {
        state->img = image_u32_create_from_pnm(getopt_get_string(gopt, "pnm"));

        printf("Loaded image %d x %d from %s\n",
               state->img->width, state->img->height,
               getopt_get_string(gopt, "pnm"));
    }

    draw(state, state->world);

    vx_remote_display_source_attr_t remote_attr;
    vx_remote_display_source_attr_init(&remote_attr);
    remote_attr.advertise_name = "Vx Demo";
    vx_remote_display_source_t * cxn = vx_remote_display_source_create_attr(&state->app, &remote_attr);
    pthread_create(&state->animate_thread, NULL, render_loop, state);

    pthread_create (&state->status_thread, NULL, status_loop, state);

    if (!getopt_get_bool(gopt,"no-gtk")) {
        gdk_threads_init ();
        gdk_threads_enter ();

        gtk_init (&argc, &argv);

        vx_gtk_display_source_t * appwrap = vx_gtk_display_source_create(&state->app);
        GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
        GtkWidget * canvas = vx_gtk_display_source_get_widget(appwrap);
        gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
        gtk_container_add(GTK_CONTAINER(window), canvas);
        gtk_widget_show (window);
        gtk_widget_show (canvas); // XXX Show all causes errors!

        g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

        gtk_main (); // Blocks as long as GTK window is open
        gdk_threads_leave ();

        vx_gtk_display_source_destroy(appwrap);

        // quit when gtk closes? Or wait for remote displays/Ctrl-C
        if (!getopt_get_bool(gopt, "stay-open"))
            state->running = 0;
    }

    pthread_join(state->animate_thread, NULL);
    vx_remote_display_source_destroy(cxn);

    lcm_destroy (state->lcm);

    state_destroy(state);
    vx_global_destroy();
    getopt_destroy(gopt);
}
