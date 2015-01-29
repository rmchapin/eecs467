#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcm_handlers.hpp"

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    maebot_laser_scan_handler handlerObject;
    lcm.subscribe("MAEBOT_LASER_SCAN", &maebot_laser_scan_handler::handleMessage, &handlerObject);
    while(0 == lcm.handle()){
        std::cout<<"utime: "<<handlerObject.get_timestamp()<<std::endl;
    }
    return 0;
}