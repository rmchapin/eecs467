#include "pose_tracker.hpp"

pose_tracker::pose_tracker()
{
	//init last calculated pose
	/*prev_best_particle.utime = 0;
	prev_best_particle.x = 0.0;
	prev_best_particle.y = 0.0;
	prev_best_particle.theta = 0.0;*/
    last_calc_pose.utime = 0;
    last_calc_pose.x = 0.0;
    last_calc_pose.y = 0.0;
    last_calc_pose.theta = 0.0;
	poses.push_back(last_calc_pose);
}

void pose_tracker::push_msg(maebot_motor_feedback_t msg, action_model & model)
{
	if (odo_msgs.size() > 0)
	{
		//push odo
        //printf("pose_tracker pushing\n");
        //printf("odo_msgs size:%d\n",odo_msgs.size());
        printf("poses size: %d\n",poses.size());
        maebot_motor_feedback_t prev_odo = odo_msgs.back();
        //printf("%d %d\n",prev_odo.encoder_left_ticks,prev_odo.encoder_right_ticks);
		odo_msgs.push_back(msg);
        //printf("pose_tracker compare time:%d %d\n",prev_odo.utime,msg.utime);
		//generate new pose in action model
		maebot_pose_t temp = model.gen_pose(poses.back(), prev_odo, odo_msgs.back());

		//push new pose
		poses.push_back(temp);
	}
	else
	{
		odo_msgs.push_back(msg);
	}
}

maebot_pose_delta_t pose_tracker::calc_deltas(int64_t t)
{	
	printf("in calc deltas");
	maebot_pose_t no1;
	maebot_pose_t no2;
	std::deque<maebot_pose_t>::iterator it;

	printf(" ,poses size:%d\n", poses.size());
	//backtrack to relevant poses
	for( it = poses.end()-1 ; it != poses.begin() ; --it ){
		if(t > it->utime){
			no1= *it;
			no2= *(it+1);
			break;
		}
	}

	//linear interolation between poses
	float total_time = float(no2.utime - no1.utime);
    float calc_time = float(t - no1.utime);
    float time_ratio = calc_time / total_time;

    float x_pos = no1.x + time_ratio*(no2.x - no1.x);
    float y_pos = no1.y + time_ratio*(no2.y - no1.y);
    float theta = no1.theta + time_ratio*(no2.theta - no1.theta);

    //calc deltas and return
	maebot_pose_delta_t ret;
	ret.utime = t;
	ret.x = x_pos - last_calc_pose.x;
	ret.y = y_pos - last_calc_pose.y;
	ret.theta = theta - last_calc_pose.theta;

	last_calc_pose.x=x_pos;
	last_calc_pose.y=y_pos;
	last_calc_pose.theta=theta;
	return ret;
}

int64_t pose_tracker::recent_pose_time()
{
	return poses.back().utime;
}

