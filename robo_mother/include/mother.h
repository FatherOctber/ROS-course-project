#ifndef MOTHER_H
#define MOTHER_H
#include "roboz.h"
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

protected:
	/**
	*  mother control scouts and drones
	**/
	void controlRobots();
};
	
#endif // MOTHER_H