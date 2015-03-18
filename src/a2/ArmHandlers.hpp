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

class BlobLCMHandler
{
    private:
        bool *blob_finished;

    public:
        BlobLCMHandler(bool *b) : blob_finished(b) {}
        ~BlobLCMHandler() { }
        void handleBlobEnd(const lcm::ReceiveBuffer *rbuf,
                               const std::string& channel,
                               const maebot_pose_t *msg)
        {
            std::cout << "blob_finished received" << std::endl;
            *blob_finished = true;
        }
};

class TurnLCMHandler
{
    private:
		bool red;
        bool *my_turn;
		int *turn_no;
int counter;

    public:
        TurnLCMHandler(bool *t, bool r, int *num_ptr)
		{
			my_turn = t;
			red = r;
			turn_no = num_ptr;
counter = 0;
		}
        ~TurnLCMHandler() { }
        void handleTurnMsg(const lcm::ReceiveBuffer *rbuf,
                               const std::string& channel,
                               const ttt_turn_t *msg)
        {
		           counter++;
if (counter%5 == 0)
{
	std::cerr << "they sent turn #" << msg->turn << std::endl;
} 
		if ((red) && (msg->turn == *turn_no))
			{            
				*my_turn = true;
			}
			else if (msg->turn > *turn_no)
			{
				*my_turn = true;
			}
        }
};

#endif
