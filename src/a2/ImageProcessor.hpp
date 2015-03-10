#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include "MagicNumbers.hpp"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_blas.h"
#include "math/gsl_util_linalg.h"
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <iostream>
#include <fstream>

class ImageProcessor
{
    public:
        ImageProcessor();
        ~ImageProcessor();

        void calculate_x();
        void read_from_file(char *filename);
        void calculate_arm_coords(gsl_vector *point, gsl_vector *world);

        void print_x();
    private:
        double A[36];
        double b[6];
        gsl_vector *x;
        gsl_matrix *T;
        
        void set_im_coord(double x, double y, int pos);
};

#endif
