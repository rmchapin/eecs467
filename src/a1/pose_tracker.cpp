#include "pose_tracker.hpp"

pose_tracker::pose_tracker()
{
	//init last calculated pose
	last_calc_pose.utime = 0;
	last_calc_pose.x = 0.0;
	last_calc_pose.y = 0.0;
	last_calc_pose.theta = 0.0;
	poses.push_back(last_calc_pose);
}

void pose_tracker::push_msg(maebot_motor_feedback_t *msg, action_model & model)
{
	odo_msgs.push_back(msg);
	//integrate to get new pose

	//push new pose	
}

maebot_pose_delta_t pose_tracker::calc_deltas(int64_t t)
{
	maebot_pose_t no1;
	maebot_pose_t no2;
	std::deque<maebot_pose_t>::iterator it;

	for( it = poses.end()-1 ; it!=poses.begin() ; --it ){
		if(t > it.utime){
			no1= *it;
			no2= *(++it);
		}
	}

	float total_time = no2.utime - no1.utime;
    float calc_time = t - no1.utime;
    float time_ratio = calc_time / total_time;

    float x_pos = no1.x + time_ratio*(no2.x - no1.x);
    float y_pos = no1.y + time_ratio*(no2.y - no1.y);
    float theta = no1.theta + time_ratio*(no2.theta - no1.theta);

	maebot_pose_delta_t ret;
	ret.utime = t;
	ret.x = x_pos - last_calc_pose.x;
	ret.y = y_pos - last_calc_pose.y;
	ret.theta = theta - last_calc_pose.theta;

	last_calc_pose = ret;
	return ret;
}

int64_t pose_tracker::recent_pose_time()
{
	return poses.back().utime;
}

void pose_tracker::integrate()
{

}
