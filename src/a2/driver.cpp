#include <iostream>
#include "ImageProcessor.hpp"

using namespace std;

int main()
{
    ImageProcessor ip;

    ip.read_from_file("calibration.txt");
    ip.calculate_x();
    ip.print_x();
    ip.print_A();
    ip.print_b();

    gsl_vector *point;
    point = gsl_vector_alloc(3);
    
    // test first point correct
    gsl_vector_set(point, 0, 791);
    gsl_vector_set(point, 1, 620);
    gsl_vector_set(point, 2, 1);
    gsl_vector *world;
    world = gsl_vector_alloc(3);
    ip.calculate_arm_coords(point, world);

    cout << "world: [" << gsl_vector_get(world, 0) << endl;
    cout << "        " << gsl_vector_get(world, 1) << endl;
    cout << "        " << gsl_vector_get(world, 2) << "]" << endl;

    // test second point correct
    gsl_vector_set(point, 0, 643);
    gsl_vector_set(point, 1, 630);
    gsl_vector_set(point, 2, 1);

    ip.calculate_arm_coords(point, world);

    cout << "world: [" << gsl_vector_get(world, 0) << endl;
    cout << "        " << gsl_vector_get(world, 1) << endl;
    cout << "        " << gsl_vector_get(world, 2) << "]" << endl;
    
    // test third point correct

    gsl_vector_set(point, 0, 655);
    gsl_vector_set(point, 1, 779);
    gsl_vector_set(point, 2, 1);

    ip.calculate_arm_coords(point, world);

    cout << "world: [" << gsl_vector_get(world, 0) << endl;
    cout << "        " << gsl_vector_get(world, 1) << endl;
    cout << "        " << gsl_vector_get(world, 2) << "]" << endl;
    
    // test fourth point(701, 681) correct
    gsl_vector_set(point, 0, 701);
    gsl_vector_set(point, 1, 681);
    gsl_vector_set(point, 2, 1);

    ip.calculate_arm_coords(point, world);

    cout << "world: [" << gsl_vector_get(world, 0) << endl;
    cout << "        " << gsl_vector_get(world, 1) << endl;
    cout << "        " << gsl_vector_get(world, 2) << "]" << endl;
    
    // test fifth point(746, 719) correct

    gsl_vector_set(point, 0, 746);
    gsl_vector_set(point, 1, 719);
    gsl_vector_set(point, 2, 1);

    ip.calculate_arm_coords(point, world);

    cout << "world: [" << gsl_vector_get(world, 0) << endl;
    cout << "        " << gsl_vector_get(world, 1) << endl;
    cout << "        " << gsl_vector_get(world, 2) << "]" << endl;
    
    // clear vectors
    gsl_vector_free(point);
    gsl_vector_free(world);
    
    return 0;
}
