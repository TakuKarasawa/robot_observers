#ifndef OBSERVER_LOADER_H_
#define OBSERVER_LOADER_H_

#include <ros/ros.h>

#include "observer_loader/observer.h"

class ObserverLoader
{
public:
	ObserverLoader() :
		private_nh_("~") {}

	ObserverLoader(ros::NodeHandle _private_nh) :
		private_nh_(_private_nh) {}

	void output(std::vector<Observer*>& observers)
	{
		observers.clear();
		std::string observer_list_name;
		private_nh_.param("OBSERVER_LIST_NAME",observer_list_name,{std::string("observer_list")});
        XmlRpc::XmlRpcValue observer_list;
        if(!private_nh_.getParam(observer_list_name.c_str(),observer_list)){
            ROS_ERROR("Cloud not load %s", observer_list_name.c_str());
            return;
        }

        ROS_ASSERT(observer_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for(int i = 0; i < (int)observer_list.size(); i++){
            if(!observer_list[i]["robot_name"].valid() || 
			   !observer_list[i]["color"].valid()){
                ROS_ERROR("%s is valid", observer_list_name.c_str());
                return;
            }
            if(observer_list[i]["robot_name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
			   observer_list[i]["color"].getType() == XmlRpc::XmlRpcValue::TypeString){
                std::string robot_name = static_cast<std::string>(observer_list[i]["robot_name"]);
				std::string color = static_cast<std::string>(observer_list[i]["color"]);
                Observer* observer (new Observer(robot_name,color));
                observers.emplace_back(observer);
            }
        }
	}

private:
	// node handler
	ros::NodeHandle private_nh_;
};

#endif  // OBSERVER_LOADER_H_