#include "ImageProcessor.hpp"

ImageProcessor::ImageProcessor()
{
    b[0] = wx1;
    b[1] = wy1;
    b[2] = wx2;
    b[3] = wy2;
    b[4] = wx3;
    b[5] = wy3;

    x = gsl_vector_alloc(6);
    T = gsl_matrix_alloc(3, 3);
}

ImageProcessor::~ImageProcessor()
{
    gsl_vector_free(x);
}

void ImageProcessor::read_from_file(const char *filename)
{
    std::ifstream input(filename);
    double x, y;
    for(int i = 0; i < 3; i++)
    {
        input >> x >> y;
        set_im_coord(x, y, i);
    }
    input.close();
}

void ImageProcessor::set_im_coord(double x, double y, int pos)
{
    int index = 12*pos;
    A[index] = x;
    A[index+1] = y;
    A[index+2] = 1;
    A[index+3] = 0;
    A[index+4] = 0;
    A[index+5] = 0;
    A[index+6] = 0;
    A[index+7] = 0;
    A[index+8] = 0;
    A[index+9] = x;
    A[index+10] = y;
    A[index+11] = 1;
}

void ImageProcessor::calculate_x()
{
    gsl_matrix_view A_matrix = gsl_matrix_view_array(A, 6, 6);
    gsl_vector_view b_vector = gsl_vector_view_array(b, 6);

    int s;
    gsl_permutation *p = gsl_permutation_alloc(6);
    gsl_linalg_LU_decomp(&A_matrix.matrix, p, &s);
    gsl_linalg_LU_solve(&A_matrix.matrix, p, &b_vector.vector, x);

    gsl_permutation_free(p);

    // set T
    gsl_matrix_set(T, 0, 0, gsl_vector_get(x, 0));
    gsl_matrix_set(T, 0, 1, gsl_vector_get(x, 1));
    gsl_matrix_set(T, 0, 2, gsl_vector_get(x, 2));
    gsl_matrix_set(T, 1, 0, gsl_vector_get(x, 3));
    gsl_matrix_set(T, 1, 1, gsl_vector_get(x, 4));
    gsl_matrix_set(T, 1, 2, gsl_vector_get(x, 5));
    gsl_matrix_set(T, 2, 0, 0);
    gsl_matrix_set(T, 2, 1, 0);
    gsl_matrix_set(T, 2, 2, 1);
}

void ImageProcessor::calculate_arm_coords(gsl_vector *point, gsl_vector *world)
{
    gslu_blas_mv (world, T, point);
}

void ImageProcessor::print_x()
{
    std::cout << "x: [" << gsl_vector_get(x, 0) << std::endl;
    std::cout << "    " << gsl_vector_get(x, 1) << std::endl;
    std::cout << "    " << gsl_vector_get(x, 2) << std::endl;
    std::cout << "    " << gsl_vector_get(x, 3) << std::endl;
    std::cout << "    " << gsl_vector_get(x, 4) << std::endl;
    std::cout << "    " << gsl_vector_get(x, 5) << "]" << std::endl;
}

// alright so state of our class at the moment:
// struct of HSV values (probably named something like....HSV because creativity is hard)
// class called ImageProcessor (?) that has a private member variable that is a vector of vectors of HSV (name..?) probably named something logical like image
// one function to read from a file/buffer and store the data in said private member variable
// a function rgbToHsv
// and a function hsvToRgb
// because at least one of those will be used in the first function
// and then a function for each task

