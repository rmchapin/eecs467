#ifndef MAEBOT_DATA
#define MAEBOT_DATA
#include <vector>
#include <math/point.hpp>
#include <deque>
//single laser
class maebot_laser{
public:    
    maebot_laser();
    maebot_laser(int64_t time, float r,float th,float inten,float x,float y);    
    int64_t get_timestamp();
    float get_range();
    float get_theta();
    float get_intensity();
    float get_x_pos();
    float get_y_pos();
    float get_x_end_pos();
    float get_y_end_pos();
private:
    int64_t utime;
    float range;
    float theta;
    float intensity;
    float x_pos;
    float y_pos;
};

class maebot_pose_delta{
public:
    maebot_pose_delta();
    maebot_pose_delta(float dx,float dy,float dt);
    float get_delta_x();
    float get_delta_y();
    float get_delta_theta();
private:
    float delta_x;
    float delta_y;
    float delta_theta;
};

#endif
