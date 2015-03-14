#include "blob.h"

// It's good form for every application to keep its state in a struct.
typedef struct state state_t;
struct state {

    getopt_t        *gopt;

    // image stuff
    char *img_url;
    image_u32_t *u32_im;

    lcm_t *lcm;
    pthread_t lcm_thread_pid;
    bool trigger;

    // threads
    //pthread_t animate_thread;
};

state_t* state;

void trigger_handler (const lcm_recv_buf_t *rbuf, const char *channel, const dynamixel_status_list_t *msg, void *user)
{
    state->trigger = true;
}

static void* run_lcm(void *input)
{
    state_t *state = input;
    dynamixel_status_list_t_subscribe (state->lcm,
                                       "BLOB_TRIGGGER",
                                       trigger_handler,
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

int
main (int argc, char *argv[])
{
    eecs467_init (argc, argv);
    state = calloc(1, sizeof(state_t));
    state->trigger = false;
    state->lcm = lcm_create (NULL);

    // Parse arguments from the command line, showing the help screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_string (state->gopt, '\0', "url", "", "Camera URL");
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

    if (strncmp(getopt_get_string(state->gopt, "file"), "", 1))
    {
        state->u32_im = image_u32_create_from_pnm(getopt_get_string(state->gopt, "file"));      
        if (!state->u32_im)
        {
            printf("specified file fails to load or does not exist!\n");
            return -2;
        }
    }
    else
    {
    	//use camera if no file specified
    	image_source_t *isrc = image_source_open (state->img_url);

	    if (isrc == NULL)
	        printf ("Error opening device.\n");
	    else {
	        // Print image format
	        image_source_format_t ifmt;
	        isrc->get_format (isrc, 0, &ifmt);
	        printf ("%3d: %4d x %4d (%s)\n",
	                    0, ifmt.width, ifmt.height, ifmt.format);
	        isrc->start (isrc);
	    }

	    image_source_format_t ifmt;
	    isrc->get_format (isrc, 0, &ifmt);

	    // Get the most recent camera frame
	    if (isrc != NULL) {
	        image_source_data_t *frmd = calloc (1, sizeof(*frmd));
	        int res = isrc->get_frame (isrc, frmd);
	        if (res < 0)
	            printf ("get_frame fail: %d\n", res);
	        else
	        {
	            state->u32_im = image_convert_u32 (frmd);
	        }
	    }
    }

    pthread_create (&state->lcm_thread_pid, NULL, run_lcm, state);

                printf("enter name for image:\n");
            char path[100];
            char *ret = fgets(path, 100, stdin);
            
            if (ret == path)
            {
                // replace \n with null character because fgets is terrible
                int len = strlen(path);
                path[len - 1] = '\0';
                strcat(path, ".pnm");
                (void) image_u32_write_pnm(state->u32_im, path);
            }

    while (1)
    {
    	if (state->trigger) //blob detection requested by AI
    	{
    		//create HSV image
    		//read bounds

    		//for each red, green, cyan
        		//create visited
        		
        		//pass 1
        		//mark regions with numbers
        		
        		//pass 2
        		//weigh regions
        		//find center of mass for sufficiently large
        		//write to file
    	}
    }

    // Cleanup
    lcm_destroy (state->lcm);
    if (state->u32_im != NULL)
        image_u32_destroy(state->u32_im);
    free(state);
}
