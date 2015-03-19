#include "Arm.hpp"

Arm::Arm() : atNextPosition(false)
{
    pthread_mutex_init(&positionMutex, NULL);
    pthread_mutex_init(&nextPositionMutex, NULL);
    pthread_cond_init(&nextPositionCV, NULL);
    for(int i = 0; i < 6; i++)
    {
        currentPosition[i] = 0;
        nextPosition[i] = 0;
    }

    //RMC - this removes warning at runtime
    fasttrig_init();
}

Arm::~Arm()
{
}

void Arm::setLCM(lcm::LCM *lcm_t)
{
    lcm = lcm_t;
}

void Arm::setCommandChannel(std::string command_channel_)
{
    command_channel = command_channel_;
}

bool Arm::getAtNextPosition()
{
    return atNextPosition;
}

void Arm::updateCurrentPosition(double pos, int index)
{
    currentPosition[index] = pos;
    pthread_mutex_lock(&nextPositionMutex);
    if(withinBounds())
    {
        atNextPosition = true;
        pthread_cond_signal(&nextPositionCV);
    }
    /*std::cout << "Current position: " << std::endl;
    for(int i = 0; i < 6; i++)
    {
        std::cout << "Servo " << i << ": " << currentPosition[i] << std::endl;
    }
    std::cout << "next Position: " << std::endl;
    for(int i = 0; i < 6; i++)
    {
        std::cout << "Servo " << i << ": " << nextPosition[i] << std::endl;
    }*/
    pthread_mutex_unlock(&nextPositionMutex);
}

bool Arm::withinBounds()
{
    for(int i = 0; i < 6; i++)
    {
        if(!withinBoundsSingle(i))
        {
            //std::cout << "servo" << i << "outside bounds!!" << std::endl;
            return false;
        }
    }
    return true;
}

bool Arm::withinBoundsSingle(int i)
{
    return eecs467::angle_diff_abs(nextPosition[i], currentPosition[i]) < M_PI/40;
}

void Arm::homeServos(bool open)
{
    for(int i = 0; i < 5; i++)
    {
        nextPosition[i] = 0;
    }

    nextPosition[5] = (open ? open_fingers : close_fingers);
}

void Arm::closeHand()
{
    nextPosition[5] = close_fingers;
}

void Arm::openHand()
{
    nextPosition[5] = open_fingers;
}

void Arm::moveToPosition(coord ballCoord, bool grab)
{
    double nextPose[6];
    calculateNextPosition(ballCoord, nextPose, grab);

    // first, move servo 0
    for(int i = 1; i < 6; i++)
    {
        nextPosition[i] = currentPosition[i];
    }
    nextPosition[0] = nextPose[0];
    publish();
    waitForMove();

    // if close position, move servo 1 to -0.6 first
    if(nextPose[1] > -0.50 && nextPose[1] < 0.50)
    {
        nextPosition[1] = -0.6;
        publish();
        waitForMove();
    }

    // move rest of servos (without closing hand)
    for(int i = 2; i < 5; i++)
    {
        nextPosition[i] = nextPose[i];
    }
    publish();
    waitForMove();

    // then move servo 1
    nextPosition[1] = nextPose[1];
    publish();
    waitForMove();
}

void Arm::grabBall(coord ballCoord)
{
    // move to ball
    moveToPosition(ballCoord, true);

    // grab ball
    closeHand();
    publish();
    waitForMove();

    // return to home
    // if arm is at one of the close positions, move back, then home servo 3, then home
    // rest of servos
    if(currentPosition[1] > -0.40 && currentPosition[1] < 0.40)
    {
        nextPosition[1] = -0.6;
    }
    else
        nextPosition[1] = 0;
    publish();
    waitForMove();

    if(nextPosition[1] != 0)
    {
        nextPosition[3] = 0;
        publish();
        waitForMove();
    }

    homeServos(false);
    publish();
    waitForMove();
}

void Arm::placeBall(coord ballCoord)
{
    // move to ball
    moveToPosition(ballCoord, false);

    // drop ball
    openHand();
    publish();
    waitForMove();

    // return to home
    // if arm is at one of the close positions, move back, then home servo 3, then home
    // rest of servos
    if(currentPosition[1] > -0.40 && currentPosition[1] < 0.40)
    {
        nextPosition[1] = -0.6;
    }
    else
        nextPosition[1] = 0;
    publish();
    waitForMove();

    if(nextPosition[1] != 0)
    {
        nextPosition[3] = 0;
        publish();
        waitForMove();
    }

    homeServos(true);
    publish();
    waitForMove();
}

void Arm::calculateNextPosition(coord ball_coord, double *nextPose, bool grab)
{
    // calculate R
    double R = dist(ball_coord);
    
	// figure out angle of ball and move there:
    nextPose[0] = rotateBase(ball_coord, R);
    nextPose[4] = 0;

	int d4 = d_4;
	if(grab)
	{
		d4 = d_4_grab;
	}

    // Calculate M
    double M = sqrt(R*R + (d4-d_1)*(d4-d_1));
    std::cout << "M: " << M << std::endl;

    if(M < (d_2+d_3)) // if not reaching too far
    {
		std::cout << "inside if" << std::endl;
        // calculate inner angles
        double alpha = atan2(d4-d_1, R);
        double beta = acos((-d_3*d_3 + d_2*d_2 + M*M)/(2*d_2*M));
        double gamma = acos((-M*M+d_2*d_2+d_3*d_3)/(2*d_2*d_3));

        std::cout << "alpha: " << alpha << std::endl;
        std::cout << "beta:  " << beta << std::endl;
        std::cout << "gamma: " << gamma << std::endl;

        // calculate servo angles
        nextPose[1] = M_PI/2 - alpha - beta;
        nextPose[2] = M_PI - gamma;
        nextPose[3] = M_PI - nextPose[1] - nextPose[2];
    }
    else
    {
        // calculate inner angles
        double f = sqrt(R*R + d_1*d_1);
        double alpha = acos((-(d_2+d_3)*(d_2+d_3) + f*f + d4*d4)/(2*f*d4));
        double beta = acos((-d4*d4 + (d_2+d_3)*(d_2+d_3) + f*f)/(2*f*(d_2+d_3)));
        double gamma = atan2(R, d_1);

        std::cout << "alpha: " << alpha << std::endl;
        std::cout << "beta:  " << beta << std::endl;
        std::cout << "gamma: " << gamma << std::endl;
        
        // calculate servo angles
        nextPose[1] = M_PI - beta - gamma;
        nextPose[2] = 0;
        nextPose[3] = alpha + beta;
    }
}

double Arm::rotateBase(coord next_coord, double R)
{
    double theta = atan2(next_coord.x, next_coord.y);
    theta = (M_PI/2) - theta;
	return theta + ((double)20/R);
}

double Arm::dist(coord next_coord)
{
    return sqrt(next_coord.x*next_coord.x + next_coord.y*next_coord.y);
}

void Arm::publish()
{
    dynamixel_command_list_t cmds;
    cmds.len = 6;
    cmds.commands.reserve(6);
    for(int i = 0; i < 6; i++)
    {
        cmds.commands[i].utime = utime_now ();
        cmds.commands[i].position_radians = nextPosition[i];
        cmds.commands[i].speed = MAX_SPEED;
        cmds.commands[i].max_torque = MAX_TORQUE;
    }
    atNextPosition = false;
    lcm->publish(command_channel.c_str(), &cmds);
}

void Arm::waitForMove()
{
    pthread_mutex_lock(&nextPositionMutex);
    while(!atNextPosition)
    {
        wait();
    }
    pthread_mutex_unlock(&nextPositionMutex);
}

void Arm::lockPositionMutex()
{
    pthread_mutex_lock(&positionMutex);
}

void Arm::unlockPositionMutex()
{
    pthread_mutex_unlock(&positionMutex);
}

void Arm::lockNextPositionMutex()
{
    pthread_mutex_lock(&nextPositionMutex);
}

void Arm::unlockNextPositionMutex()
{
    pthread_mutex_unlock(&nextPositionMutex);
}

void Arm::wait()
{
    pthread_cond_wait(&nextPositionCV, &nextPositionMutex);
}
