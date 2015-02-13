#ifndef ACTION_MODEL_HPP
#define ACTION_MODEL_HPP

#include <stdint.h>
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "math/gsl_util_rand.h"

class action_model{
public: 
    action_model();
    action_model(float e1, float e2);
    void init_model(maebot_motor_feedback_t prev_odo, maebot_motor_feedback_t curr_odo);
    maebot_pose_t gen_pose(maebot_pose_t prev_pose);
private:
    gsl_rng* rand_gen;
    float k1;
    float k2;
    float dis;
    float dtheta;
    float alpha;
    int64_t curr_timestamp;
};

#endif
