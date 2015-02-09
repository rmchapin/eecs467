#ifndef ACTION_MODEL_HPP
#define ACTION_MODEL_HPP

#include <stdint.h>
#include <random>
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"


class action_model{
public: 
    action_model();
    action_model(float e1, float e2);
    void init_model(maebot_motor_feedback_t prev,maebot_motor_feedback_t curr);
    maebot_pose_t get_new_pose(maebot_pose_t prev_pose);

private:
    std::mt19937 rand_gen;
    std::normal_distribution<float> nor_dist;
    float k1;
    float k2;
    float dis;
    float dtheta;
    float alpha;   
    int64_t curr_timestamp;
};

#endif
