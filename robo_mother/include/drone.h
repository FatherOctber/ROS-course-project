#ifndef ROBOTS_H
#define ROBOTS_H
#include <ros/ros.h>
#include "roboz.h"
using namespace Roboz;

class Drone: public AbstractMoveableRobot
{
public:
	Drone(ros::NodeHandle& node, const std::string& model_path, const std::string& name);
	Drone(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z);
	virtual ~Drone();
	virtual void execute();
	virtual void robotCallback(const robo_mother::command::ConstPtr& message);
		
protected:
	virtual void move(double dx, double dy, double dz);
	virtual void mine(double atX, double atY, double atZ); // command drone to do mining at position
		
};	

#endif // ROBOTS_H
