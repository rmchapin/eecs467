#ifndef ARM_HPP
#define ARM_HPP

#include "MagicNumbers.hpp"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_blas.h"
#include "math/gsl_util_linalg.h"
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"

#include <iostream>
#include <fstream>
#include <cmath>

struct ArmPosition
{
    double theta0;
    double theta1;
    double theta2;
    double theta3;
    double theta4;
    double theta5;
};

struct coord
{
    double x;
    double y;
};

class Arm
{
    private:
        ArmPosition currentPosition;
        dynamixel_command_list_t cmds;

        lcm::LCM *lcm;
        
        double rotateBase(coord next_coord);
        double dist(coord next_coord);
    public:
        Arm(lcm::LCM *lcm_t);
        Arm(); // for testing only
        ~Arm();

        ArmPosition nextPose(coord ball_coord);
};

#endif
