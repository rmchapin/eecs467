#include "ip.h"

// It's good form for every application to keep its state in a struct.
typedef struct state state_t;
struct state {
    bool running;
    getopt_t        *gopt;
    parameter_gui_t *pg;
    
    //A2 stuff
    int mode; // which mode of operation are we currently in
              // mode = 1: mask mode
              // mode = 2: color picker mode
              // mode = 3: calibration mode
    bool capture; //has image been captured from camera
    image_u32_t *u32_im;
    image_u32_t *revert;
    int fsm_counter;
    
    double x1, x2, x3, y1, y2, y3;
    int cal_counter;

    double Hmin, Hmax, Smin, Smax, Vmin, Vmax;
    HSV_p cp_array[5];
    int cp_index;
    double last_click_x;
    double last_click_y;

    // image stuff
    char *img_url;
    int   img_height;
    int   img_width;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;    // where vx objects are live
    vx_event_handler_t *vxeh;       // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;

    // threads
    pthread_t animate_thread;

    // for accessing the arrays
    //pthread_mutex_t mutex;
};

state_t * state_create (void);
void state_destroy (state_t *state);

void clear_all(state_t *state)
{
    //reset fsm counter
    state->fsm_counter = 0;
    //clear mask points
    //clear color picker array
    //zero color picker index
    state->cp_index = 0;
    state->Hmin = -1.0;
    state->Hmax = -1.0;
    state->Smin = -1.0;
    state->Smax = -1.0;
    state->Vmin = -1.0;
    state->Vmax = -1.0;
    //clear calibration points
}

// === Parameter listener =================================================
// This function is handed to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// state, etc if need be.
static void
my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    state_t *state = pl->impl;
           
    if (0 == strcmp("cb1", name))
    {
        if (pg_gb(pg,name))
        {
            printf("masking mode selected\n");
            clear_all(state);
            state->mode = 1;
            pg_sb(pg,"cb2",0);
            pg_sb(pg,"cb3",0);
        }
    }
    else if (0 == strcmp("cb2", name))
    {
        if (pg_gb(pg,name))
        {
            printf("color picker selected\n");
            clear_all(state);
            state->mode = 2;
            pg_sb(pg,"cb1",0);
            pg_sb(pg,"cb3",0);
        }
    }
    else if (0 == strcmp("cb3", name))
    {
        if (pg_gb(pg,name))
        {
            printf("calibration mode selected\n");
            clear_all(state);
            state->mode = 3;
            pg_sb(pg,"cb1",0);
            pg_sb(pg,"cb2",0);
        }
    }
    else if (0 == strcmp("but1", name))
    {
        //grab image from camera and save internally
        state->capture = 1;
    }
    else if (0 == strcmp("but2", name))
    {
        if (pg_gb(pg, "cb1") || pg_gb(pg,"cb2") || pg_gb(pg,"cb3"))
        {
            if (state->capture)
            {
                //advance state machine to next part of whatever mode
                if (state->mode == 1)
                {
                    //mask
                }
                else if (state->mode == 2)
                {
                    //convert vx image coords to pixel index
                    //RMC - figure this^ out
                    //convert pixel to HSV
                    //store in array
                    //compare to bounds
                    state->fsm_counter++;
                    state->cp_index++;
                }
                else if (state->mode == 3)
                {
                    //calibration
                }

            }
            else
            {
                printf("you must CAPTURE an image!\n");
            }
        }
        else
        {
            printf("you must select a MODE!\n");
        }
    }
    else if (0 == strcmp("but3", name))
    {
        printf("resetting application\n");
        
        state->capture = 0;
        state->mode = -1;
        pg_sb(pg,"cb1",0);
        pg_sb(pg,"cb2",0);
        pg_sb(pg,"cb3",0);

        clear_all(state);
    }
    else //invalid
        printf("INVALID parameter change\n");
}

static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    state_t *state = vxeh->impl;

    // vx_camera_pos_t contains camera location, field of view, etc
    // vx_mouse_event_t contains scroll, x/y, and button click events

    if ((mouse->button_mask & VX_BUTTON1_MASK) &&
        !(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {

        vx_ray3_t ray;
        vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);

        double ground[3];
        vx_ray3_intersect_xy (&ray, 0, ground);

        printf ("Mouse clicked at coords: [%8.3f, %8.3f]  Ground clicked at coords: [%6.3f, %6.3f]\n",
                mouse->x, mouse->y, ground[0], ground[1]);
    }

    // store previous mouse event to see if the user *just* clicked or released
    state->last_mouse_event = *mouse;

    return 0;
}

static int
key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
    state_t *state = vxeh->impl;
    
    if (key->key_code == VX_KEY_s)
    {
        if (key->released)
        {
            if (state->capture)
            {
                printf("imaged saved as \"cam_image.pnm\"\n");
                (void) image_u32_write_pnm(state->u32_im, "cam_image.pnm");

                //RMC - fix this for user selected image saves
                printf("enter name for image:\n");
                char path[100];
                char *ret = fgets(path, 100, stdin);
                
                printf("path is : %s\n", path);
                if (ret == path)
                {
                    char path[1024];

                    // replace \n with null character because fgets is terrible
                    int len = strlen(path);
                    path[len - 2] = '\0';
                    
                    strcat(path, "pnm");
                    (void) image_u32_write_pnm(state->u32_im, path);
                }
            }
        }
    }

    return 0;
}

void * animate_thread (void *data)
{
    const int fps = 60;
    state_t *state = data;

    // Set up the imagesource
    image_source_t *isrc = image_source_open (state->img_url);

    if (isrc == NULL)
        printf ("Error opening device.\n");
    else {
        // Print out possible formats. If no format was specified in the
        // url, then format 0 is picked by default.
        // e.g. of setting the format parameter to format 2:
        //
        // --url=dc1394://bd91098db0as9?fidx=2
        for (int i = 0; i < isrc->num_formats (isrc); i++) {
            image_source_format_t ifmt;
            isrc->get_format (isrc, i, &ifmt);
            printf ("%3d: %4d x %4d (%s)\n",
                    i, ifmt.width, ifmt.height, ifmt.format);
        }
        isrc->start (isrc);
    }

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) {

        // Get the most recent camera frame and render it to screen.
        if (isrc != NULL) {
            image_source_data_t *frmd = calloc (1, sizeof(*frmd));
            int res = isrc->get_frame (isrc, frmd);
            if (res < 0)
                printf ("get_frame fail: %d\n", res);
            else {
                // Handle frame
                //image_u32_t *im = image_convert_u32 (frmd);
                if (state->u32_im)
                {
                    image_u32_destroy(state->u32_im);
                }
                state->u32_im = image_convert_u32 (frmd);
                state->revert = image_convert_u32 (frmd);
                if (state->u32_im != NULL) {
                    vx_object_t *vim = vxo_image_from_u32(state->u32_im,
                                                          VXO_IMAGE_FLIPY,
                                                          VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

                    // render the image centered at the origin and at a normalized scale of +/-1 unit in x-dir
                    const double scale = 2./state->u32_im->width;
                    vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "image"),
                                        vxo_chain (vxo_mat_scale3 (scale, scale, 1.0),
                                                   vxo_mat_translate3 (-state->u32_im->width/2., -state->u32_im->height/2., 0.),
                                                   vim));
                    vx_buffer_swap (vx_world_get_buffer (state->vxworld, "image"));
                }
            }
            fflush (stdout);
            isrc->release_frame (isrc, frmd);
        }

        //continue displaying captured image
        while (state->capture)
        {
            if (state->u32_im != NULL)
            {
                vx_object_t *vim = vxo_image_from_u32 (state->u32_im,
                                                       VXO_IMAGE_FLIPY,
                                                       VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

                    // render the image centered at the origin and at a normalized scale of +/-1 unit in x-dir
                    const double scale = 2./state->u32_im->width;
                    vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "image"),
                                        vxo_chain (vxo_mat_scale3 (scale, scale, 1.0),
                                                   vxo_mat_translate3 (-state->u32_im->width/2., -state->u32_im->height/2., 0.),
                                                   vim));
                    vx_buffer_swap (vx_world_get_buffer (state->vxworld, "image"));
            }

            usleep (1000000/fps);
        }

        usleep (1000000/fps);
    }

    if (isrc != NULL)
        isrc->stop (isrc);

    return NULL;
}

state_t *
state_create (void)
{
    state_t *state = calloc (1, sizeof(*state));

    state->vxworld = vx_world_create ();
    state->vxeh = calloc (1, sizeof(*state->vxeh));
    state->vxeh->key_event = key_event;
    state->vxeh->mouse_event = mouse_event;
    state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);

    state->mode = -1; //start in no mode
    state->capture = 0;
    state->fsm_counter = 0;
    state->Hmin = -1.0;
    state->Hmax = -1.0;
    state->Smin = -1.0;
    state->Smax = -1.0;
    state->Vmin = -1.0;
    state->Vmax = -1.0;
    state->cp_index = 0;

    state->cal_count = 0;

    state->running = 1;

    return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

    if (state->vxeh)
        free (state->vxeh);

    if (state->gopt)
        getopt_destroy (state->gopt);

    if (state->pg)
        pg_destroy (state->pg);

    free (state);
}

// This is intended to give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.
int
main (int argc, char *argv[])
{
    eecs467_init (argc, argv);
    state_t *state = state_create ();


    printf("here\n");

    // Parse arguments from the command line, showing the help screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_string (state->gopt, '\0', "url", "", "Camera URL");
    getopt_add_bool   (state->gopt,  'l', "list", 0, "Lists available camera URLs and exit");
    getopt_add_string (state->gopt,  'f', "file", "", "Image file to load");

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--url=CAMERAURL] [--file=IMAGEPATH] [other options]\n\n", argv[0]);
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }

    // Set up the imagesource. This looks for a camera url specified on
    // the command line and, if none is found, enumerates a list of all
    // cameras imagesource can find and picks the first url it finds.
    if (strncmp (getopt_get_string (state->gopt, "url"), "", 1)) {
        state->img_url = strdup (getopt_get_string (state->gopt, "url"));
        printf ("URL: %s\n", state->img_url);
    }
    else {
        // No URL specified. Show all available and then use the first
        zarray_t *urls = image_source_enumerate ();
        printf ("Cameras:\n");
        for (int i = 0; i < zarray_size (urls); i++) {
            char *url;
            zarray_get (urls, i, &url);
            printf ("  %3d: %s\n", i, url);
        }

        if (0==zarray_size (urls)) {
            printf ("Found no cameras.\n");
            return -1;
        }

        zarray_get (urls, 0, &state->img_url);
    }

    if (getopt_get_bool (state->gopt, "list")) {
        state_destroy (state);
        exit (EXIT_SUCCESS);
    }

    if (strncmp(getopt_get_string(state->gopt, "file"), "", 1))
    {
        printf("loading from file not currently supported\n");
        return -2;

        state->u32_im = image_u32_create_from_pnm(getopt_get_string(state->gopt, "file"));      
        if (!state->u32_im)
        {
            printf("specified file fails to load or does not exist!\n");
            return -2;
        }

        state->capture = 1;
    }

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    // Initialize a parameter gui
    state->pg = pg_create ();
    pg_add_check_boxes (state->pg,
                        "cb1", "Mask Mode", 0,
                        "cb2", "Color Picker Mode", 0,
                        "cb3", "Calibration Mode", 0,
                        NULL);
    pg_add_buttons (state->pg,
                    "but1", "Capture Image",
                    "but2", "Advance",
                    "but3", "Reset",
                    NULL);

    parameter_listener_t *my_listener = calloc (1, sizeof(*my_listener));
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener (state->pg, my_listener);

    // Launch our worker threads
    pthread_create (&state->animate_thread, NULL, animate_thread, state);

    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, 1024, 768);

    // Quit when GTK closes
    state->running = 0;
    pthread_join (state->animate_thread, NULL);

    // Cleanup
    free (my_listener);
    state_destroy (state);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();
}
