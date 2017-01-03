#ifndef ROBOZ_H
#define ROBOZ_H
#include <ros/ros.h>
#include <string>
#include <robo_mother/command.h>
#include <robo_mother/robotstate.h>


namespace Roboz {

	class AllStateItem {
	public:
		int id;
		int robottype;
		double x;
		double y;
		int state;
	};

	enum GlobalState { Idle, Online, Offline }; // Global state of world condition: if Offline - close program
	
	class IMoveable
	{
	public:
		IMoveable() {};
		virtual ~IMoveable() {};
		virtual void move(double dx, double dy, double distance)=0;
	};
	
	class AbstractObject 
	{
		
	public:
		AbstractObject(ros::NodeHandle& node, const std::string& name);
		AbstractObject(ros::NodeHandle& node, const std::string& name, double x, double y, double z);
		virtual ~AbstractObject();
		double getX();
		double getY();
		double getZ();
		void setPosition(double x, double y, double z);
		bool isOnlineState();
		void turnOn(bool switchIt); //false - go offline, true (by default set) - go online
		std::string getName();
		virtual void spawn()=0; // create self on XY-position
		
	protected:
		double mX;
		double mY;
		double mZ;
		std::string name;
		GlobalState state;
		int id;
		ros::NodeHandle node;
	};
	
	
	class AbstractRobot: public AbstractObject
	{
	public:
		AbstractRobot(ros::NodeHandle& node, const std::string& model_path, const std::string& name);
		AbstractRobot(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z);
		virtual ~AbstractRobot();
		virtual void spawn();
		virtual void execute()=0; // life-cycle
		virtual void robotCallback(const robo_mother::command::ConstPtr& message)=0;
		
	protected:
		ros::Publisher robotstatePublisher;
		std::string model;
	};

	class AbstractMoveableRobot: public AbstractRobot, IMoveable
	{
	public:
		AbstractMoveableRobot(ros::NodeHandle& node, const std::string& model_path, const std::string& name);
		AbstractMoveableRobot(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z);
		virtual ~AbstractMoveableRobot();
		virtual void move(double dx, double dy, double distance);
		double getDistancePrecision();
		double getCurrentAngle();

	protected:
		double mDistancePrecision;
	    double mAngularPrecision;
	    double mCurrentAngle;
	    double mLinearSpeed;
	    double mAngularSpeed;
	    double mMaxLinearSpeed;
	    virtual double getRotateAngle(double dx, double dy);
	};

}

#endif // ROBOZ_H