#include "robot_observers/robot_observers.h"

RobotObservers::RobotObservers() :
    private_nh_("~"), 
    loader_(new ObserverLoader(private_nh_))
{
    private_nh_.param("HZ",HZ_,{10});
}

void RobotObservers::generate_observer()
{
    std::vector<Observer*> obs;
    loader_->output(obs);
    for(const auto &o : obs){
        // std::cout << "robot_name: " << o->robot_name << std::endl;
        // std::cout << "color: " << o->color << std::endl;
        RobotObserver* robot_observer (new RobotObserver(nh_,private_nh_,o->robot_name));
        observers_.emplace_back(robot_observer);
    }
    for(const auto &o : observers_) o->debug();
}

void RobotObservers::process()
{
    generate_observer();
    ros::Rate rate(HZ_);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}
