#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_u8.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "eecs467_util.h"    // This is where a lot of the internals live

typedef struct {
	unsigned char a;
	unsigned char b;
	unsigned char g;
	unsigned char r;
} ABGR_p;

typedef struct {
	double h;
	double s;
	double v;
} HSV_p;

typedef struct {
    int x;
    int y;
} pix_coord;

int max(int a, int b)
{
    if(a>b)
        return a;
    return b;
}

int min(int a, int b)
{
    if(a<b)
        return a;
    return b;
}

HSV_p u32_pix_to_HSV(ABGR_p u32_in)
{
	HSV_p out;
	double min, max, delta;

    min = u32_in.r < u32_in.g ? u32_in.r : u32_in.g;
    min = min  < u32_in.b ? min  : u32_in.b;
    
    max = u32_in.r > u32_in.g ? u32_in.r : u32_in.g;
    max = max  > u32_in.b ? max  : u32_in.b;
    
    out.v = max / 255.0;                        // v
    delta = max - min;
    if( max > 0.0 )
    {
        out.s = (delta / max);                  // s
    }
    else
    {
        // r = g = b = 0                        // s = 0, v is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    
    if( u32_in.r >= max )                        // > is bogus, just keeps compilor happy
    {
        out.h = ( u32_in.g - u32_in.b ) / delta;        // between yellow & magenta
    }
    else
    {
        if( u32_in.g >= max )
            out.h = 2.0 + ( u32_in.b - u32_in.r ) / delta;  // between cyan & yellow
        else
            out.h = 4.0 + ( u32_in.r - u32_in.g ) / delta;  // between magenta & cyan
    }
    
    out.h *= 60.0;                              // degrees
    
    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}