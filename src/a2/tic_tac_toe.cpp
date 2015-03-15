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

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/math_util.h"

#include "MagicNumbers.hpp"
#include "Arm.hpp"
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

    pthread_t status_thread;
    pthread_t command_thread;
    pthread_t decision_thread;

    dynamixel_command_list_t cmds;
    
    Arm *arm;
    ImageProcessor *ip;
};

void *
command_loop (void *data)
{
    state_t *state = (state_t *) data;
    const int hz = 30;

    while (1) {
        // Send LCM commands to arm. Normally, you would update positions, etc,
        // but here, we will just home the arm.
        if (getopt_get_bool (state->gopt, "idle")) {
            for (int id = 0; id < NUM_SERVOS; id++) {
                state->cmds.commands[id].utime = utime_now ();
                state->cmds.commands[id].position_radians = 0.0;
                state->cmds.commands[id].speed = 0.0;
                state->cmds.commands[id].max_torque = 0.0;
            }
            state->lcm->publish(state->command_channel.c_str(), &state->cmds);
        }
        else {
            coord balls[9];
            balls[0].x = ballx3; balls[0].y = bally3 - y_test_offset;
            balls[1].x = ballx3; balls[1].y = bally2 - y_test_offset;
            balls[2].x = ballx3; balls[2].y = bally1 - y_test_offset;
            balls[3].x = ballx2; balls[3].y = bally3 - y_test_offset;
            balls[4].x = ballx2; balls[4].y = bally2 - y_test_offset;
            balls[5].x = ballx2; balls[5].y = bally1 - y_test_offset;
            balls[6].x = ballx1; balls[6].y = bally3 - y_test_offset;
            balls[7].x = ballx1; balls[7].y = bally2 - y_test_offset;
            balls[8].x = ballx1; balls[8].y = bally1 - y_test_offset;

            // home servos slowly
            state->arm->homeServos(true);
            state->arm->publish();
            state->arm->waitForMove();

            for(int i = 0; i < 9; i++)
            {
                // pick up ball
                state->arm->grabBall(balls[i]);

                // place ball
                state->arm->placeBall(balls[i]);
            }
        }

        usleep (1000000/hz);
    }

    return NULL;
}

void *
decision_loop (void *user)
{
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
    getopt_add_bool (gopt, 'i', "idle", 0, "Command all servos to idle");
    getopt_add_string (gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");
    getopt_add_string (gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");

    if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help")) {
        getopt_do_usage (gopt);
        exit (EXIT_FAILURE);
    }

    state_t *state = new state_t;
    state->arm = new Arm;
    state->ip = new ImageProcessor;
    state->lcm = new lcm::LCM;
    state->cmds.len = NUM_SERVOS;
    state->cmds.commands.reserve(6);

    state->gopt = gopt;
    state->command_channel = std::string(getopt_get_string (gopt, "command-channel"));
    state->status_channel = std::string(getopt_get_string (gopt, "status-channel"));
    
    state->arm->setLCM(state->lcm);
    state->arm->setCommandChannel(state->command_channel);

    ArmLCMHandler lcm_handler(state->arm);

    state->lcm->subscribe(state->status_channel,
                          &ArmLCMHandler::handleArmPosition,
                          &lcm_handler);
    
    pthread_create (&state->command_thread, NULL, command_loop, state);
    pthread_create (&state->decision_thread, NULL, decision_loop, state);

    // Loop forever
    while(state->lcm->handle() == 0);

    // Probably not needed, given how this operates
    pthread_join (state->status_thread, NULL);
    pthread_join (state->command_thread, NULL);
    pthread_join (state->decision_thread, NULL);

    delete state;
    getopt_destroy (gopt);
}
