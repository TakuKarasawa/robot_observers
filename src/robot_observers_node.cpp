#include "robot_observers/robot_observers.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"robot_observers");
    RobotObservers robot_observers;
    robot_observers.process();
    return 0;
}
