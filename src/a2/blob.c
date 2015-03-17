#include "blob.h"

// It's good form for every application to keep its state in a struct.
typedef struct state state_t;
struct state {

    getopt_t        *gopt;

    // image stuff
    bool cam;
    char *img_url;
    image_u32_t *u32_im;
    HSV_p *hsv_im;

    lcm_t *lcm;
    pthread_t lcm_thread_pid;
    bool trigger;
    
    RANGE_t bounds[3];
    int *record;

    pix_coord output[3][10];

    // threads
    //pthread_t animate_thread;
};

state_t* state;

void trigger_handler (const lcm_recv_buf_t *rbuf, const char *channel, const maebot_pose_t *msg, void *user)
{
    state->trigger = true;
}

static void* run_lcm(void *input)
{
    state_t *state = input;
    maebot_pose_t_subscribe (state->lcm,
                             "BLOB_TRIGGER",
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

bool within_range(RANGE_t range, HSV_p p)
{
    if ((p.h >= range.Hmin) && (p.h <= range.Hmax) &&
        (p.s >= range.Smin) && (p.s <= range.Smax) &&
        (p.v >= range.Vmin) && (p.v <= range.Vmax))
        {
            return true;
        }
    return false;
}

int
main (int argc, char *argv[])
{
    eecs467_init (argc, argv);
    state = calloc(1, sizeof(state_t));
    state->trigger = false;
    state->cam = true;
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
        state->cam = false;
        state->u32_im = image_u32_create_from_pnm(getopt_get_string(state->gopt, "file"));      
        if (!state->u32_im)
        {
            printf("specified file fails to load or does not exist!\n");
            return -2;
        }
        else
        {
            printf("image loaded from %s\n", getopt_get_string(state->gopt, "file"));
        }
    }

    //setup camera
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

    pthread_create (&state->lcm_thread_pid, NULL, run_lcm, state);

    //read mask
    FILE * fptr = fopen("mask.txt", "r");
    if (fptr == NULL)
    {
        printf("mask.txt does not exist, or could not be opened\n");
        exit(-2);
    }
    int mask_x1, mask_x2, mask_y1, mask_y2;
    fscanf(fptr, "%d %d %d %d", &mask_x1, &mask_y1, &mask_x2, &mask_y2);
    fclose(fptr);

    int x_dim = mask_x2-mask_x1;
    int y_dim = mask_y2-mask_y1;
    printf("x_dim: %d, y_dim: %d\n", x_dim, y_dim);
    printf("read mask.txt\n");

    if (!state->cam)
    {
        //create HSV image
        state->hsv_im = malloc(x_dim*y_dim*sizeof(HSV_p));
        int g, h;
        for (h = 0; h < y_dim; h++)
        {
            for (g = 0; g < x_dim; g++)
            {
                ABGR_p pixel_abgr;
                uint32_t val = state->u32_im->buf[(mask_y1 + h)*state->u32_im->stride + (mask_x1 + g)];
                //state->u32_im->buf[(mask_y1 + h)*state->u32_im->stride + (mask_x1 + g)] = 0xFFE600CB;
                pixel_abgr.a = 0xFF & (val >> 24);
                pixel_abgr.b = 0xFF & (val >> 16);
                pixel_abgr.g = 0xFF & (val >> 8);
                pixel_abgr.r = 0xFF & val;
                  
                state->hsv_im[(h*x_dim) + g] = u32_pix_to_HSV(pixel_abgr);
            }
        }
        printf("created HSV\n");
    }

    //read bounds
    char* inputs[] = {"cyan.txt", "green.txt", "red.txt"};
    int in = 0;
    for (in = 0; in < 3; in++)
    {
        fptr = fopen(inputs[in], "r");
        if (fptr == NULL)
        {
            printf("%s does not exist, or could not be opened\n", inputs[in]);
            exit(-3);
        }
        fscanf(fptr, "%lf %lf %lf %lf %lf %lf", &state->bounds[in].Hmin, &state->bounds[in].Hmax, &state->bounds[in].Smin, &state->bounds[in].Smax, &state->bounds[in].Vmin, &state->bounds[in].Vmax);
        fclose(fptr);
    }

    for (in=0; in < 3; in++)
    {
        printf("bounds[%d]: %lf, %lf, %lf, %lf, %lf, %lf\n", in, state->bounds[in].Hmin, state->bounds[in].Hmax, state->bounds[in].Smin, state->bounds[in].Smax, state->bounds[in].Vmin, state->bounds[in].Vmax);
    }

    //while (1)
    {
    	int hz;
        hz = 10;

        if (1)//(state->trigger) //blob detection requested by AI
    	{
            //clear output
			int k, t;
			for (k = 0; k < 3; k++)
			{
				for (t = 0; t < 5; t++)
				{
					state->output[k][t].x = -1;
					state->output[k][t].y = -1;
				}
			}

            if (state->cam)
            {
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

                //create HSV image
                if (state->hsv_im)
                {
                    free(state->hsv_im);
                }

                state->hsv_im = malloc(x_dim*y_dim*sizeof(HSV_p));
                int g, h;
                for (h = 0; h < y_dim; h++)
                {
                    for (g = 0; g < x_dim; g++)
                    {
                        ABGR_p pixel_abgr;
                        uint32_t val = state->u32_im->buf[(mask_y1 + h)*state->u32_im->stride + (mask_x1 + g)];
                        pixel_abgr.a = 0xFF & (val >> 24);
                        pixel_abgr.b = 0xFF & (val >> 16);
                        pixel_abgr.g = 0xFF & (val >> 8);
                        pixel_abgr.r = 0xFF & val;
                          
                        state->hsv_im[(h*x_dim) + g] = u32_pix_to_HSV(pixel_abgr);
                    }
                }
            }
    		
            //for each cyan, green, red
            for (in = 0; in < 3; in++)
            {
                //printf("searching for color %d\n", in);
                state->record = calloc(x_dim*y_dim, sizeof(int));
                Stack *S = malloc(sizeof(Stack));
                int blob_num = 1;
                int x_avg = 0;
                int y_avg = 0;
                int count = 0;
				int store_index = 0;

                //traverse hsv image area
                int g, h;
                for (h = 0; h < y_dim; h++)
                {
                    for (g = 0; g < x_dim; g++)
                    {
                        //if not yet examined
                        if (state->record[(h*x_dim) + g] == 0)
                        {
                            //if in the color range
                            if (within_range(state->bounds[in], state->hsv_im[(h*x_dim) + g]))
                            {
                                //mark visited, push
                                state->record[(h*x_dim) + g] = blob_num;
                                pix_coord push;
                                push.x = g;
                                push.y = h;
                                Stack_Push(S, push);

                                //still visiting current blob
                                while (!Stack_Empty(S))
                                {
                                    //pop
                                    pix_coord visit;
                                    visit = Stack_Top(S);
                                    Stack_Pop(S);

                                    //increment counters
                                    count++;
                                    x_avg += visit.x;
                                    y_avg += visit.y;

                                    //check all neighbors
                                    int n, m;
                                    for (n = -1; n < 2; n++)
                                    {
                                        for (m = -1; m < 2; m++)
                                        {
                                            //bound checking, prevent segfaults
                                            if ((visit.x + n >= 0) && (visit.x + n < x_dim) && (visit.y + m >= 0) && (visit.y + m < y_dim))
                                            {
                                                //if not yet visited
                                                if (state->record[((visit.y + m)*x_dim) + visit.x + n] == 0)
                                                {
                                                    //if in color range, push
                                                    if (within_range(state->bounds[in], state->hsv_im[((visit.y + m)*x_dim) + visit.x + n]))
                                                    {
                                                        state->u32_im->buf[(mask_y1+visit.y+m)*state->u32_im->stride + (mask_x1+visit.x+n)] = 0xFFE600CB;
                                                        state->record[((visit.y + m)*x_dim) + visit.x + n] = blob_num;
                                                        pix_coord push;
                                                        push.x = visit.x + n;
                                                        push.y = visit.y + m;
                                                        Stack_Push(S, push);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                if (count > 50)
                                {
                                    printf("blob %d of color %d found at %d,%d\n", store_index, in, (x_avg / count) + mask_x1, (y_avg / count) + mask_y1);
                                    state->output[in][store_index].x = x_avg / count;
                                    state->output[in][store_index].y = y_avg / count;
									store_index++;
                                }

                                blob_num++;
                                count = 0;
                                x_avg = 0;
                                y_avg = 0;
                            }
                        }
                    }
                }

                free(state->record);
                free(S);

				//write to output_ file
				switch (in)
				{
					case 2:
						(void) image_u32_write_pnm(state->u32_im, "output_blob.pnm");
					break;
				}		
			}

			//write output to file
			FILE * fptr = fopen("blob_output.txt", "w");
			if (fptr == NULL)
			{
	    		printf("blob_output.txt or could not be opened\n");
	    		exit(-2);
			}
			else
			{
				int r, d, x, y;
				r = d = 0;
				while (r < 3)
				{
					fprintf(fptr, "#");				
					while (d < 5)
					{
						x = state->output[r][d].x;
						y = state->output[r][d].y;
						
						if ((x != -1) && (y != -1))
						{
							fprintf(fptr, " %d %d", x + mask_x1, y + mask_y1);							
						}
						d++;
					}					

					fprintf(fptr, "\n");
					d = 0;
					r++;
				}
			}
			fclose(fptr);	

            //send lcm mesage blobdone
			maebot_pose_t send;
			send.utime = 0;
    		send.x = 0.0;
			send.y = 0.0;
    		send.theta = 0.0;
			maebot_pose_t_publish (state->lcm, "BLOB_DONE", &send);

        }//if trigger

        usleep (1000000/hz);
    }

	while (1)
	{
		continue;
	}

    // Cleanup
    lcm_destroy (state->lcm);
    if (state->u32_im != NULL)
        image_u32_destroy(state->u32_im);
    free(state);
}
