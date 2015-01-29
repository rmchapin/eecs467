#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "lcm_handlers.hpp"

int main(int argc, char** argv)
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    MaebotPoseHandler handlerObject;
    lcm.subscribe("MAEBOT_POSE", &MaebotPoseHandler::handleMessage, &handlerObject);
    while(0 == lcm.handle());
    return 0;
}