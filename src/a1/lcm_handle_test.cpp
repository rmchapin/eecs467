#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include <pthread.h>
#include "lcm_handlers.hpp"
#include <lcmtypes/maebot_pose_t.hpp>

class State{
public:
    lcm::LCM lcm;
    FILE *fp_test;
    maebot_pose_handler pose_handler;
};

State* state;

void * run_lcm(void * input){
    //State * state = (State*) input;
    while(1){
        state->lcm.handle();
        //fprintf(state->fp_test,"%f %f %f\n",state->pose_handler.get_x_pos(),state->pose_handler.get_y_pos(),state->pose_handler.get_theta());
    } 
}

static void pose_data_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_pose_t *msg, State* state){
    int res = system ("clear");
    //fprintf(state->fp_test,"%f %f %f\n",msg->x,msg->y,msg->theta);
    printf("%f\n",msg->x);
}

int main(int argc, char** argv)
{
	//state_t * state = (state_t * ) calloc(1, sizeof(state_t));
    //State* state = new State;
    state = new State;
    //state->fp_test = fopen("test.txt","w");
    //state->lcm = lcm::LCM();
    //state.lcm = new lcm::LCM();
    if(!state->lcm.good()){
        printf("lcm not good, exit\n");
        exit(1);
        //return 1;
    }
    //state->lcm->subscribe("MAEBOT_POSE", &maebot_pose_handler::handleMessage, &state->pose_handler);
    state->lcm.subscribeFunction("MAEBOT_POSE",pose_data_handler,state);
    //while(0 == state.lcm.handle()){
    //    std::cout<<"utime: "<<state.pose_handler.get_timestamp()<<std::endl;
    //}
    pthread_t lcm_thread_pid;
    pthread_create(&lcm_thread_pid,NULL,run_lcm,NULL);
    //while(true){
    //    state->lcm.handle();
    //}
    //delete state->lcm;
    while(1);
    delete state;
    return 0;
}
