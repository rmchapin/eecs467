#ifndef MAEBOT_LOCALIZATION_UTIL
#define MAEBOT_LOCALIZATION_UTIL

#include <deque>

maebot_pose_delta calc_deltas(std::deque<maebot_motor_feedback_t> &odometry, int64_t start_time, int64_t end_time);

#endif
