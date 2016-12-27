#ifndef MOTHER_H
#define MOTHER_H
#include "roboz.h"
#include <map>
using namespace Roboz;

/**
 *  mother robot (commander)
 **/
class Mother: public AbstractRobot 
{
public:
	Mother(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z);
	virtual ~Mother();
	virtual void execute();
	virtual void robotCallback(const robo_mother::command::ConstPtr& message);
	virtual void robotstateCallback(const robo_mother::robotstate::ConstPtr& message);
	virtual void resourcewantedCallback(const robo_mother::robotstate::ConstPtr& message);
protected:
	int resources;
	std::map<int, AllStateItem> allrobots;
	ros::Publisher resourceFoundPublisher;
	/**
	*  mother control scouts and drones
	**/
	void controlRobots();
};
	
#endif // MOTHER_H