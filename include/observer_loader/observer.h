#ifndef OBSERVER_H_
#define OBSERVER_H_

#include <iostream>

class Observer
{
public:
	Observer() :
		robot_name(std::string("")), color(std::string("")) {}

	Observer(std::string _robot_name,std::string _color) :
		robot_name(_robot_name), color(_color) {}

	std::string robot_name;
	std::string color;

private:
};

#endif	// OBSERVER_H_