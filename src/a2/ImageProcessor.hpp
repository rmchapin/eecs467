#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include "MagicNumbers.hpp"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_blas.h"
#include "math/gsl_util_linalg.h"

#include <iostream>
#include <fstream>

class ImageProcessor
{
    private:
        double A[36];
        double b[6];
        gsl_vector *x;
        gsl_matrix *T;
        
        void set_im_coord(double x, double y, int pos);
    public:
        ImageProcessor();
        ~ImageProcessor();

        void calculate_x();
        void read_from_file(const char *filename);
        void calculate_arm_coords(gsl_vector *point, gsl_vector *world);

        void print_x();
        void print_A();
        void print_b();
};

#endif
