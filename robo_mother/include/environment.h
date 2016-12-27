#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H
#include "roboz.h"
using namespace Roboz;

/**
 *  environment block object;
 *  notation - every in-gazebo object is robot
 **/
class StandartBlock: public AbstractRobot 
{
public:
	StandartBlock(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z);
	virtual ~StandartBlock();
	virtual void execute();
	virtual void robotCallback(const robo_mother::command::ConstPtr& message);
};

/**
 *  environment resource object;
 *  notation - every in-gazebo object is robot
 **/
class Resource: public AbstractRobot 
{
public:
	Resource(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z, int size = 100);
	virtual ~Resource();
	int getSize(); // resource balance size
	int getRandomPart(); //get part of resource
	virtual void execute();
	virtual void robotCallback(const robo_mother::command::ConstPtr& message);
	virtual void decreaseCallback(const robo_mother::robotstate::ConstPtr& message);
protected:
	ros::Subscriber decreaseSubscriber;
	ros::Publisher decreasePublisher;
private:
	int resources;
	double rotationAngle;
};
	
#endif // ENVIRONMENT_H