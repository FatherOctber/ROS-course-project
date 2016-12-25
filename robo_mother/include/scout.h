#ifndef SCOUT_H
#define SCOUT_H
#include <ros/ros.h>
#include "roboz.h"
using namespace Roboz;

class Scout: public AbstractMoveableRobot
{
public:
	Scout(ros::NodeHandle& node, const std::string& model_path, const std::string& name);
	Scout(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z);
	virtual ~Scout();
	virtual void execute();
	virtual void robotCallback(const robo_mother::command::ConstPtr& message);
		
protected:
	virtual void explore(double atX, double atY); // command to do exploration
		
};	

#endif // SCOUT_H
