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
	virtual void motheranswerCallback(const robo_mother::robotstate::ConstPtr& message);
	virtual void decreaseFromResCallback(const robo_mother::robotstate::ConstPtr& message);
	virtual void motherIncrementCallback(const robo_mother::robotstate::ConstPtr& message);
protected:
	virtual void mine(double atX, double atY, double atZ); // command drone to do mining at position
	virtual void waitForAnswerFromMother();
	virtual void moveToTarget();
	virtual void takeResource();
	virtual void moveToHome();
	virtual void giveResource();

	bool isMotherAnswered;
	AllStateItem* targetItem;
	int* takenRes;

	ros::Publisher gazeboPublisher;
	ros::Publisher succeedPublisher;
	ros::Subscriber succeedSupscriber;
	ros::Publisher resourceWantedPublisher;
	ros::Publisher resourceDecreasePublisher;
	ros::Subscriber wannaAnswerSubscriber;
	ros::Subscriber wannaDecreaseSubscriber;
};	

#endif // ROBOTS_H
