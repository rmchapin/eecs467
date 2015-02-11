#ifndef ACTION_MODEL_HPP
#define ACTION_MODEL_HPP

#include <stdint.h>
#include <random>
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include <chrono>
#include <math.h>

class action_model{
public: 
    action_model();
    action_model(float e1, float e2);

    maebot_pose_t gen_pose(maebot_pose_t prev_pose, maebot_motor_feedback_t prev_odo,maebot_motor_feedback_t curr_odo);

private:
    std::mt19937 rand_gen;
    //std::normal_distribution<float> nor_dist;
    float k1;
    float k2;
};

#endif
