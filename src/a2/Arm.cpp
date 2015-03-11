#include "Arm.hpp"

Arm::Arm(lcm::LCM *lcm_t) : lcm(lcm_t)
{
}

Arm::Arm()
{
}

Arm::~Arm()
{
}

ArmPosition Arm::nextPose(coord ball_coord)
{
    // figure out angle of ball and move there:
    ArmPosition nextPose;
    nextPose.theta0 = rotateBase(ball_coord);
    nextPose.theta5 = -nextPose.theta0;

    // calculate R
    double R = dist(ball_coord);

    // Calculate M
    double M = sqrt(R*R + (d_4+h_-d_1)*(d_4+h_-d_1));
    assert(M == M);
    assert(M != 0);
    std::cout << "M: " << M << std::endl;

    // calculate inner angles
    double alpha = atan2(d_4+h_-d_1, R);
    double beta = acos((-d_3*d_3 + d_2*d_2 + M*M)/(2*d_2*M));
    double gamma = acos((-M*M+d_2*d_2+d_3*d_3)/(2*d_2*d_3));

    std::cout << "alpha: " << alpha << std::endl;
    std::cout << "beta:  " << beta << std::endl;
    std::cout << "gamma: " << gamma << std::endl;

    // calculate servo angles
    nextPose.theta1 = M_PI/2 - alpha - beta;
    nextPose.theta2 = M_PI - gamma;
    nextPose.theta3 = M_PI - nextPose.theta1 - nextPose.theta2;

    return nextPose;
}

double Arm::rotateBase(coord next_coord)
{
    return asin(next_coord.x/dist(next_coord));
}

double Arm::dist(coord next_coord)
{
    return sqrt(next_coord.x*next_coord.x + next_coord.y*next_coord.y);
}
