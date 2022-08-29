#ifndef ROBOT_OBSERVER_H_
#define ROBOT_OBSERVER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "observer_loader/observer_loader.h"
#include "observation_result_publisher/observation_result_publisher.h"

// Custom msg
#include "object_color_detector_msgs/ObjectColorPosition.h"
#include "object_color_detector_msgs/ObjectColorPositions.h"

class RobotObserver
{
public:
    RobotObserver() :
        private_nh_("~") ,
        publisher_(new ObservationResultPublisher(nh_,private_nh_)),
        has_received_pose_(false), ROBOT_NAME_(std::string("")) { setup_subscriber(); }

    RobotObserver(ros::NodeHandle _nh,ros::NodeHandle _private_nh,std::string _robot_name) :
        nh_(_nh), private_nh_(_private_nh),
        publisher_(new ObservationResultPublisher(nh_,private_nh_)),
        has_received_pose_(false), ROBOT_NAME_(_robot_name) { setup_subscriber(); }

    void debug()
    {
        publisher_->print_observer();
    }

private:
    void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
    {
        pose_ = *msg;
        has_received_pose_ = true;
    }

    void ocp_callback(const object_color_detector_msgs::ObjectColorPositionsConstPtr& msg)
    {
        if(has_received_pose_){
            if(msg->object_color_position.size() == 0) return;
            for(const auto &m : msg->object_color_position){
                if(m.probability < 0.7) continue;
            }
            has_received_pose_ = false;
        } 
    }

    void setup_subscriber()
    { 
        std::string pose_topic_name = ROBOT_NAME_ + "/amcl_pose";
        pose_sub_ = nh_.subscribe(pose_topic_name,1,&RobotObserver::pose_callback,this); 

        std::string ocp_topic_name = ROBOT_NAME_  + "/oc_pose";
        ocp_sub_ = nh_.subscribe(ocp_topic_name,1,&RobotObserver::ocp_callback,this);
    }    

    // node handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // subscriber
    ros::Subscriber pose_sub_;
    ros::Subscriber ocp_sub_;

    // publishers
    // std::vector<ObservationResultPublisher*> publishers_;
    ObservationResultPublisher* publisher_;

    // buffer
    geometry_msgs::PoseWithCovarianceStamped pose_;
    object_color_detector_msgs::ObjectColorPositions obs_;

    // dynamic parameters
    bool has_received_pose_;

    // robot name
    std::string ROBOT_NAME_;
};

#endif  // ROBOT_OBSERVER_H_
