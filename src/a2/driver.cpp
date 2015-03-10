#include <iostream>
#include "ImageProcessor.hpp"

using namespace std;

int main()
{
    ImageProcessor ip;

    ip.read_from_file("calibration.txt");
    ip.calculate_x();
    ip.print_x();

    gsl_vector *point;
    gsl_vector_set(point, 0, 0);
    gsl_vector_set(point, 1, 0);
    gsl_vector_set(point, 2, 0);
    gsl_vector_set(point, 3, 0);
    gsl_vector_set(point, 4, 0);
    gsl_vector_set(point, 5, 0);
    gsl_vector *world;
    ip.calculate_arm_coords(point, world);

    cout << "world: [" << gsl_vector_get(world, 0) << endl;
    cout << "        " << gsl_vector_get(world, 1) << endl;
    cout << "        " << gsl_vector_get(world, 2) << endl;
    cout << "        " << gsl_vector_get(world, 3) << endl;
    cout << "        " << gsl_vector_get(world, 4) << endl;
    cout << "        " << gsl_vector_get(world, 5) << "]" << endl;

    gsl_vector_free(point);
    gsl_vector_free(world);
    
    return 0;
}
