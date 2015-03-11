#include "Arm.hpp"
#include <iostream>

using namespace std;

int main()
{
    Arm arm;
    
    // first pose:
    coord next_coord = {216, -89};
    ArmPosition next_pos = arm.nextPose(next_coord);
    
    cout << "first pose: " << endl;
    cout << "servo 0: " << next_pos.theta0 << endl;
    cout << "servo 1: " << next_pos.theta1 << endl;
    cout << "servo 2: " << next_pos.theta2 << endl;
    cout << "servo 3: " << next_pos.theta3 << endl;
    cout << "servo 4: " << next_pos.theta4 << endl;
    cout << "servo 5: " << next_pos.theta5 << endl;

    // second pose:
    next_coord.x = 38;
    next_coord.y = -89;
    next_pos = arm.nextPose(next_coord);
    
    cout << "second pose: " << endl;
    cout << "servo 0: " << next_pos.theta0 << endl;
    cout << "servo 1: " << next_pos.theta1 << endl;
    cout << "servo 2: " << next_pos.theta2 << endl;
    cout << "servo 3: " << next_pos.theta3 << endl;
    cout << "servo 4: " << next_pos.theta4 << endl;
    cout << "servo 5: " << next_pos.theta5 << endl;

    // third pose:
    next_coord.x = 38;
    next_coord.y = 89;
    next_pos = arm.nextPose(next_coord);
    
    cout << "third pose: " << endl;
    cout << "servo 0: " << next_pos.theta0 << endl;
    cout << "servo 1: " << next_pos.theta1 << endl;
    cout << "servo 2: " << next_pos.theta2 << endl;
    cout << "servo 3: " << next_pos.theta3 << endl;
    cout << "servo 4: " << next_pos.theta4 << endl;
    cout << "servo 5: " << next_pos.theta5 << endl;

    // fourth pose:
    next_coord.x = 95;
    next_coord.y = -25;
    next_pos = arm.nextPose(next_coord);
    
    cout << "fourth pose: " << endl;
    cout << "servo 0: " << next_pos.theta0 << endl;
    cout << "servo 1: " << next_pos.theta1 << endl;
    cout << "servo 2: " << next_pos.theta2 << endl;
    cout << "servo 3: " << next_pos.theta3 << endl;
    cout << "servo 4: " << next_pos.theta4 << endl;
    cout << "servo 5: " << next_pos.theta5 << endl;

    // fifth pose:
    next_coord.x = 155;
    next_coord.y = 25;
    next_pos = arm.nextPose(next_coord);
    
    cout << "fifth pose: " << endl;
    cout << "servo 0: " << next_pos.theta0 << endl;
    cout << "servo 1: " << next_pos.theta1 << endl;
    cout << "servo 2: " << next_pos.theta2 << endl;
    cout << "servo 3: " << next_pos.theta3 << endl;
    cout << "servo 4: " << next_pos.theta4 << endl;
    cout << "servo 5: " << next_pos.theta5 << endl;

    return 0;
}
