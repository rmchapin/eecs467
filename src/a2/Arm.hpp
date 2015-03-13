#ifndef ARM_HPP
#define ARM_HPP

#include "MagicNumbers.hpp"
#include "math/gsl_util_matrix.h"
#include "math/gsl_util_vector.h"
#include "math/gsl_util_blas.h"
#include "math/gsl_util_linalg.h"
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "pthread.h"
#include "common/timestamp.h"
#include <sys/select.h>
#include <sys/time.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

struct coord
{
    double x;
    double y;
};

class Arm
{
    private:
        double currentPosition[6];
        pthread_mutex_t positionMutex;
        
        double nextPosition[6];
        pthread_mutex_t nextPositionMutex;
        pthread_cond_t nextPositionCV;
        bool atNextPosition;
        
        dynamixel_command_list_t cmds;

        lcm::LCM *lcm;
        std::string command_channel;
        
        double rotateBase(coord next_coord);
        double dist(coord next_coord);
    public:
        Arm(); // for testing only
        ~Arm();

        void moveToNextPosition(coord ball_coord);
        void setLCM(lcm::LCM *lcm_t);
        void setCommandChannel(std::string command_channel_);
        void updateCurrentPosition(double pos, int index);
        bool withinBounds();
        bool withinBoundsSingle(int i);
        void publish();

        // movement functions
        void homeServos(bool open);
        bool getAtNextPosition();
        void closeHand();
        void openHand();

        void lockPositionMutex();
        void unlockPositionMutex();
        void lockNextPositionMutex();
        void unlockNextPositionMutex();
        void wait();
};

#endif
