#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <stdio.h>
class MaebotPoseHandler
{
public:
    ~MaebotPoseHandler() {}
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
        const std::string& channel,
        const maebot_pose_t* msg)
    {
        printf("Received Message on channel %s\n",channel.c_str());
        //printf("utime: %lld\n" (long long) msg->utime);
        printf("x: %f, y: %f, theta: %f\n",msg->x,msg->y,msg->theta);
    }
};

class LaserScanHandler
{
    ~LaserScanHandler() {}
    void handleMessage(const lcm::ReceiveBuffer* rbuf,
        const std::string& channel,
        const maebot_laser_scan_t* msg)
    {
        //printf("Received Message on channel %s\n",channel.c_str());
        //printf("utime: %lld\n" (long long) msg->utime);
        //printf("x: %f, y: %f, theta: %f\n",msg->x,msg->y,msg->theta);
    }
};