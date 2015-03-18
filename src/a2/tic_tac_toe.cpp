#include <cassert>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <cmath>
#include <sys/select.h>
#include <sys/time.h>
#include <pthread.h>
#include <string>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/dynamixel_command_list_t.hpp"
#include "lcmtypes/dynamixel_command_t.hpp"
#include "lcmtypes/dynamixel_status_list_t.hpp"
#include "lcmtypes/dynamixel_status_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/ttt_turn_t.hpp"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/math_util.h"

#include "MagicNumbers.hpp"
#include "Arm.hpp"
#include "Board.hpp"
#include "ImageProcessor.hpp"
#include "ArmHandlers.hpp"

#define NUM_SERVOS 6
#define ERROR_MARGIN 0.1
#define ANGLE_MAX_TO 0.6
#define ANGLE_MIN_TO -0.6

typedef struct state state_t;
struct state
{
    getopt_t *gopt;

    // LCM
    lcm::LCM *lcm;
    std::string command_channel;
    std::string status_channel;
	std::string turn_send_channel;
	std::string turn_rec_channel;

    pthread_t command_thread;
    pthread_t turn_thread;

    dynamixel_command_list_t cmds;

	//control
	int turn_num;
	bool my_turn;
	bool blob_finished;
	bool am_i_red;
    
    Arm *arm;
	Board *board;
    ImageProcessor *ip;
};

void *
command_loop (void *data)
{
    state_t *state = (state_t *) data;
    const int hz = 10;

	state->board->printInit();
	state->arm->homeServos(true);
	state->arm->publish();
	state->arm->waitForMove();

    while (1)
	{
        if (state->my_turn)
		{
			std::cout << "it's our turn" << std::endl;
			state->my_turn = false;

			//trigger blob
			maebot_pose_t send;
			send.utime = utime_now();
			send.x = 0.0;
			send.y = 0.0;
			send.theta = 0.0;
			state->lcm->publish("BLOB_TRIGGER", &send);
			
			while(!state->blob_finished)
			{
				//nothing
			}
			state->blob_finished = false;

			//board update
			state->board->updateBoard("blob_output.txt");
			state->board->print();

			//if board game over
			if (state->board->gameOver())
			{
				std::cout << "game complete!" << std::endl;
				exit(0);
			}
			else
			{
				//state->board->nextPick();
				//state->board->nextPlace();
				state->arm->grabBall(state->board->nextPick());
				state->arm->placeBall(state->board->nextPlace());
				state->turn_num++;
			}
			
			std::cout << "make your move, HUMAN!" << std::endl;
			std::string trash;
			std::cin >> trash;
			state->my_turn = true;
		}
	
        usleep (1000000/hz);
    }

    return NULL;
}

void *
turn_loop (void *data)
{
	state_t *state = (state_t *) data;

	const int hz = 20;

	while (1)
	{
		ttt_turn_t t;
		t.utime = utime_now();
		t.turn = state->turn_num;
		state->lcm->publish(state->turn_send_channel.c_str(), &t);

		usleep(1000000/hz);
	}    

	return NULL;
}

// This subscribes to the status messages sent out by the arm, displaying servo
// state in the terminal. It also sends messages to the arm ordering it to the
// "home" position (all servos at 0 radians).
int
main (int argc, char *argv[])
{
    getopt_t *gopt = getopt_create ();
    getopt_add_bool (gopt, 'h', "help", 0, "Show this help screen");
	getopt_add_bool (gopt, 'r', "red", 0, "Am I red player?");
    getopt_add_bool (gopt, 'i', "idle", 0, "Command all servos to idle");

    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt);
        exit (EXIT_FAILURE);
    }

    state_t *state = new state_t;
    state->arm = new Arm;
    state->ip = new ImageProcessor;
    state->ip->read_from_file("calibration.txt");
    state->ip->print_A();
    state->ip->print_b();
    state->ip->calculate_x();
    state->ip->print_x();
    state->lcm = new lcm::LCM;
    state->cmds.len = NUM_SERVOS;
    state->cmds.commands.reserve(6);

    state->gopt = gopt;
    state->command_channel = std::string("ARM_COMMAND");
    state->status_channel = std::string("ARM_STATUS");

	if (getopt_get_bool(gopt, "red"))
	{
		std::cout << "player is RED" << std::endl;
		state->am_i_red = true;
		state->my_turn = true;
		state->turn_send_channel = "RED_TURN";
		state->turn_rec_channel = "GREEN_TURN";
	}
	else
	{
		std::cout << "player is GREEN" << std::endl;
		state->am_i_red = false;
		state->my_turn = false;
		state->turn_send_channel = "GREEN_TURN";
		state->turn_rec_channel = "RED_TURN";
	}

	state->board = new Board(state->ip, state->am_i_red);
	state->board->boardInit("blob_output.txt");

    state->arm->setLCM(state->lcm);
    state->arm->setCommandChannel(state->command_channel);

    if (getopt_get_bool (state->gopt, "idle"))
	{
        std::cout << "setting arm to idle..." << std::endl;
        for (int id = 0; id < NUM_SERVOS; id++)
		{
            state->cmds.commands[id].utime = utime_now ();
            state->cmds.commands[id].position_radians = 0.0;
            state->cmds.commands[id].speed = 0.0;
            state->cmds.commands[id].max_torque = 0.0;
		}
	    state->lcm->publish(state->command_channel.c_str(), &state->cmds);
		exit(0);
	}

    ArmLCMHandler arm_handler(state->arm);
    state->lcm->subscribe(state->status_channel,
                          &ArmLCMHandler::handleArmPosition,
                          &arm_handler);

	state->blob_finished = false;    
	BlobLCMHandler blob_handler(&state->blob_finished);
    state->lcm->subscribe("BLOB_DONE",
                          &BlobLCMHandler::handleBlobEnd,
                          &blob_handler);

	TurnLCMHandler turn_handler(&state->my_turn, state->am_i_red, &state->turn_num);
    state->lcm->subscribe(state->turn_rec_channel,
                          &TurnLCMHandler::handleTurnMsg,
                          &turn_handler);

    pthread_create (&state->command_thread, NULL, command_loop, state);
    pthread_create (&state->turn_thread, NULL, turn_loop, state);

    // Loop forever
    while(state->lcm->handle() == 0);

    // Probably not needed, given how this operates
    pthread_join (state->command_thread, NULL);
    pthread_join (state->turn_thread, NULL);

    delete state;
    getopt_destroy (gopt);
}
