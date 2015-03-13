#ifndef ARM_HANDLERS_HPP
#define ARM_HANDLERS_HPP

#include "Arm.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"

class ArmLCMHandler
{
    private:
        Arm *arm;

    public:
        ArmLCMHandler(Arm *a) : arm(a) { }
        ~ArmLCMHandler() { }
        void handleArmPosition(const lcm::ReceiveBuffer *rbuf,
                               const std::string& channel,
                               const dynamixel_status_list_t *msg)
        {
            arm->lockPositionMutex();
            for(int i = 0; i < 6; i++)
            {
                arm->updateCurrentPosition(msg->statuses[i].position_radians, i);
            }
            arm->unlockPositionMutex();
        }
};

#endif
