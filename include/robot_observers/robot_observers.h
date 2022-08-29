#ifndef ROBOT_OBSERVERS_H
#define ROBOT_OBSERVERS_H

#include <ros/ros.h>

#include "observation_result_publisher/observation_result_publisher.h"
#include "robot_observer/robot_observer.h"
#include "observer_loader/observer_loader.h"

class RobotObservers
{
public:
    RobotObservers();
    void process();

private:
    void generate_observer();

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // observers
    std::vector<RobotObserver*> observers_;

    // list loader
    ObserverLoader* loader_;

    // parameters
    int HZ_;
};

#endif  // ROBOT_OBSERVERS_H
