#include <vector>
#include <math/point.hpp>
class maebot_pose_data{
public:
    maebot_pose_data();
    maebot_pose_data(const maebot_pose_data& other);
    maebot_pose_data(int64_t utime,float x,float y,float theta);
    int64_t get_timestamp();
    float get_x_pos();
    float get_y_pos();
    float get_theta();
    ~maebot_pose_data(){}
private:
    int64_t pose_time;
    float pose_x_curr;
    float pose_y_curr;
    float pose_theta_curr;
};

class maebot_laser_data{
public:
    maebot_laser_data();
    maebot_laser_data(int64_t utime,int32_t nranges,std::vector<float> rs,std::vector<float> ths,std::vector<int64_t> ts,std::vector<float> is);
    maebot_laser_data(int64_t utime,int32_t nranges,std::vector<float> rs,std::vector<float> ths,std::vector<int64_t> ts,std::vector<float> is,maebot_pose_data pose);
    void calc_end_points();
    int64_t get_timestamp();
    int32_t get_num_ranges();
    std::vector<float> get_ranges();
    std::vector<float> get_thetas();
    std::vector<int64_t> get_times();
    std::vector<float> get_intensities();
    maebot_pose_data get_curr_pose();
    std::vector<eecs467::Point<float>> get_end_points(); 
private:
    int64_t laser_time;
    int32_t num_ranges;
    std::vector<float> ranges;
    std::vector<float> thetas;
    std::vector<int64_t> times;
    std::vector<float> intensities;
    maebot_pose_data curr_pose;
    std::vector<eecs467::Point<float>> end_points;
};

