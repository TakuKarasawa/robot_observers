#ifndef OBSERVATION_RESULT_PUBLISHER_H_
#define OBSERVATION_RESULT_PUBLISHER_H_

#include <ros/ros.h>

// loader
#include "observer_loader/observer_loader.h"

// Custom msg
#include "multi_robot_msgs/ObservedRobotPose.h"

class ObservationResultPublisher : public std::map<Observer*,ros::Publisher*>
{
public:
    ObservationResultPublisher() :
        private_nh_("~"), 
        loader_(new ObserverLoader(private_nh_)),
        MAP_FRAME_ID_(std::string("map")) { load(); }

    ObservationResultPublisher(ros::NodeHandle _nh,ros::NodeHandle _private_nh) :
        nh_(_nh), private_nh_(_private_nh),
        loader_(new ObserverLoader(private_nh_)),
        MAP_FRAME_ID_(std::string("map")) { load(); }

    void print_observer()
    {
        for(auto it = this->begin(); it != this->end(); it++){
            std::cout << "robot_name: " << it->first->robot_name << std::endl;
            std::cout << "color: " << it->first->color << std::endl;
        }
        std::cout << std::endl;
    }

    std::string get_color(std::string robot_name)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->first->robot_name == robot_name) return it->first->color;
        }
        return std::string("");
    }

    std::string get_robot_name(std::string color)
    {
        for(auto it = this->begin(); it != this->end(); it++){
            if(it->first->color == color) return it->first->robot_name;
        }
        return std::string("");
    }

    void publish_msg(std::string robot_name,multi_robot_msgs::Pose _pose)
    {
        multi_robot_msgs::ObservedRobotPose pose;
        pose.header.frame_id = MAP_FRAME_ID_;
        pose.header.stamp = ros::Time::now();
        pose.pose = _pose;

        for(auto it = this->begin(); it != this->end(); it++){
            if(it->first->robot_name == robot_name) it->second->publish(pose);
        }
    }

    void publish_msg(std::string robot_name,double x,double y,double yaw,double weight)
    {
        multi_robot_msgs::Pose pose;
        pose.x = x;
        pose.y = y;
        pose.yaw = yaw;
        pose.weight = weight;

        publish_msg(robot_name,pose);
    }

private:
    void load()
	{
		this->clear();
        std::vector<Observer*> obs;
        loader_->output(obs);
        for(const auto &o : obs){
            Observer* observer (new Observer(o->robot_name,o->color));
            ros::Publisher* obs_pub (new ros::Publisher());
            std::string obs_topic_name = o->robot_name + "/obs_pose";
            *obs_pub = nh_.advertise<multi_robot_msgs::ObservedRobotPose>(obs_topic_name,1);
            this->insert(std::map<Observer*,ros::Publisher*>::value_type(observer,obs_pub)); 
        }
	}

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // loader
    ObserverLoader* loader_;

    // parameters
    std::string MAP_FRAME_ID_;
};

#endif  // OBSERVATION_RESULT_PUBLISHER_H_