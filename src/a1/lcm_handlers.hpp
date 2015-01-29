#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <stdio.h>
#include <vector>
class maebot_pose_handler
{
public:
    ~maebot_pose_handler() {}
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
        const std::string& channel,
        const maebot_pose_t* msg)
    {
        utime = msg->utime;
        x = msg->x;
        y = msg->y;
        theta = msg->theta;
    }
    int64_t get_timestamp(){
        return utime;
    }
    float get_x_pos(){
        return x;
    }
    float get_y_pos(){
        return y;
    }
    float get_theta(){
        return theta;
    }
private:
    int64_t utime;
    float x;
    float y;
    float theta;
};

class maebot_laser_scan_handler
{
public:
    ~maebot_laser_scan_handler(){}
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
        const std::string& channel,
        const maebot_laser_scan_t* msg)
    {
        utime = msg->utime;
        num_ranges = msg->num_ranges;
        for (int i = 0; i < num_ranges; ++i) {
            ranges.push_back(msg->ranges[i]);
            thetas.push_back(msg->thetas[i]);
            times.push_back(msg->times[i]);
            intensities.push_back(msg->intensities[i]);
        }
    }
    int64_t get_timestamp(){
        return utime;
    }
    int32_t get_num_ranges(){
        return num_ranges;
    }
    std::vector<float> get_ranges(){
        return ranges;
    }
    std::vector<float> get_thetas(){
        return thetas;
    }
    std::vector<int64_t> get_times(){
        return times;
    }
    std::vector<float> get_intensities(){
        return intensities;
    }
private:
    int64_t    utime;
    int32_t    num_ranges;
    std::vector< float > ranges;
    std::vector< float > thetas;
    std::vector< int64_t > times;
    std::vector< float > intensities;
};