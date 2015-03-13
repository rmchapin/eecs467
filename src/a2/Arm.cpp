#include "Arm.hpp"

Arm::Arm() : atNextPosition(false)
{
    pthread_mutex_init(&positionMutex, NULL);
    pthread_mutex_init(&nextPositionMutex, NULL);
    pthread_cond_init(&nextPositionCV, NULL);
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
        std::cout << "INSIDE BOUNDS!!" << std::endl;
        atNextPosition = true;
        pthread_cond_signal(&nextPositionCV);
    }
    pthread_mutex_unlock(&nextPositionMutex);
}

bool Arm::withinBounds()
{
    for(int i = 0; i < 6; i++)
    {
        if(!withinBoundsSingle(i))
        {
            std::cout << "servo" << i << "outside bounds!!" << std::endl;
            return false;
        }
    }
    return true;
}

bool Arm::withinBoundsSingle(int i)
{
    return abs(nextPosition[i] - currentPosition[i]) < M_PI/30;
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

void Arm::moveToNextPosition(coord ball_coord)
{
    // figure out angle of ball and move there:
    pthread_mutex_lock(&nextPositionMutex);
    nextPosition[0] = rotateBase(ball_coord);
    nextPosition[4] = -nextPosition[0];
    pthread_mutex_unlock(&nextPositionMutex);

    // calculate R
    double R = dist(ball_coord) - 5;

    // Calculate M
    double M = sqrt(R*R + (d_4-d_1)*(d_4-d_1));
    std::cout << "M: " << M << std::endl;

    if(M < (d_2+d_3)) // if not reaching too far
    {
        // calculate inner angles
        double alpha = atan2(d_4-d_1, R);
        double beta = acos((-d_3*d_3 + d_2*d_2 + M*M)/(2*d_2*M));
        double gamma = acos((-M*M+d_2*d_2+d_3*d_3)/(2*d_2*d_3));

        std::cout << "alpha: " << alpha << std::endl;
        std::cout << "beta:  " << beta << std::endl;
        std::cout << "gamma: " << gamma << std::endl;

        // calculate servo angles
        pthread_mutex_lock(&nextPositionMutex);
        nextPosition[1] = M_PI/2 - alpha - beta;
        nextPosition[2] = M_PI - gamma;
        nextPosition[3] = M_PI - nextPosition[1] - nextPosition[2];
        pthread_mutex_unlock(&nextPositionMutex);
    }
    else
    {
        // calculate inner angles
        double f = sqrt(R*R + d_1*d_1);
        double alpha = acos((-(d_2+d_3)*(d_2+d_3) + f*f + d_4*d_4)/(2*f*d_4));
        double beta = acos((-d_4*d_4 + (d_2+d_3)*(d_2+d_3) + f*f)/(2*f*(d_2+d_3)));
        double gamma = atan2(R, d_1);

        std::cout << "alpha: " << alpha << std::endl;
        std::cout << "beta:  " << beta << std::endl;
        std::cout << "gamma: " << gamma << std::endl;
        
        // calculate servo angles
        pthread_mutex_lock(&nextPositionMutex);
        nextPosition[1] = M_PI - beta - gamma;
        nextPosition[2] = 0;
        nextPosition[3] = alpha + beta;
        pthread_mutex_unlock(&nextPositionMutex);
    }
}

double Arm::rotateBase(coord next_coord)
{
    double theta = atan2(next_coord.x, next_coord.y);
    return (3*M_PI/2) - theta;
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
