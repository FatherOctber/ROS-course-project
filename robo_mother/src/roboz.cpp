#include "roboz.h"
#include <gazebo_msgs/SpawnModel.h>
#include <fstream>
#include <cmath>
#include <time.h>

namespace Roboz {
	
	AbstractObject::AbstractObject(ros::NodeHandle& node, const std::string& name)
	{
		this->node = node;
		this->name = name.substr(1,name.length()-1);
		mX = 0.0;
		mY = 0.0;
		this->state = GlobalState::Idle;

		std::hash<std::string> hash;
		id = hash(name);
		std::cout<<"current id " << id << std::endl;
	}
	
	AbstractObject::AbstractObject(ros::NodeHandle& node, const std::string& name, double x, double y, double z): AbstractObject(node, name)
	{
		std::hash<std::string> hash;
		id = hash(name);
		std::cout<<"cur id " << id << std::endl;
		setPosition(x, y, z);
	}
	
	AbstractObject::~AbstractObject()
	{
		//todo
	}
	
	double AbstractObject::getX()
	{
		return mX;
	}
	
	double AbstractObject::getY()
	{
		return mY;	
	}
	
	double AbstractObject::getZ()
	{
		return mZ;	
	}
	
	void AbstractObject::setPosition(double x, double y, double z)
	{
		mX = x;
		mY = y;
		mZ = z;
	}
	
	bool AbstractObject::isOnlineState() 
	{
		if(state == GlobalState::Online) {
			return true;
		} else {
			return false;
		}
	}
	
	void AbstractObject::turnOn(bool switchIt)
	{
		if(switchIt) {
			state = GlobalState::Online;
		} else {
			state = GlobalState::Offline;
		}
	}
	
	std::string AbstractObject::getName()
	{
		return name;
	}
	
	AbstractRobot::AbstractRobot(ros::NodeHandle& node, const std::string& model_path, const std::string& name): AbstractObject(node, name)
	{
		model = model_path;
	}
	
	AbstractRobot::AbstractRobot(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z): AbstractObject(node, name, x, y, z) 
	{
		model = model_path;
	}
	
    AbstractRobot::~AbstractRobot() 
	{
		//todo
	}
	
    void AbstractRobot::spawn()
	{
		ros::service::waitForService("gazebo/spawn_sdf_model");
		ros::ServiceClient add_robot = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
		gazebo_msgs::SpawnModel srv;
		std::ifstream fin(model.c_str());
		std::string model_xml;
		std::string buf;
		while(!fin.eof())
		{
			getline(fin, buf);
			model_xml += buf + "\n";
		}
		srv.request.model_xml = model_xml;
		srv.request.model_name = name;
		geometry_msgs::Pose pose;
		pose.position.x = mX;
		pose.position.y = mY;
		pose.position.z = mZ;
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;

		srv.request.initial_pose = pose;
		add_robot.call(srv);
	}

	AbstractMoveableRobot::AbstractMoveableRobot(ros::NodeHandle& node, const std::string& model_path, const std::string& name)
		: AbstractRobot(node, model_path, name)
	{
		mDistancePrecision = 0.01;
		mAngularPrecision = 1.0 * M_PI / 180;
		mCurrentAngle = 0.0;
      	mLinearSpeed = 0.1;
      	mAngularSpeed = 0.1; 
      	mMaxLinearSpeed = 0.05;
	}
	
	AbstractMoveableRobot::AbstractMoveableRobot(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z)
		: AbstractRobot(node, model_path, name, x, y, z) 
	{
		mDistancePrecision = 0.01;
		mAngularPrecision = 1.0 * M_PI / 180;
		mCurrentAngle = 0.0;
      	mLinearSpeed = 0.1;
      	mAngularSpeed = 0.1; 
      	mMaxLinearSpeed = 0.05;
	}
	
	AbstractMoveableRobot::~AbstractMoveableRobot() 
	{
		//todo
	}

	double AbstractMoveableRobot::getDistancePrecision() 
	{
		return mDistancePrecision;
	}

	double AbstractMoveableRobot::getCurrentAngle() 
	{
		return mCurrentAngle;
	}

	void AbstractMoveableRobot::move(double dx, double dy, double distance)
	{
		// update coords
	    double currentLinearSpeed = mLinearSpeed * distance;
	    if (currentLinearSpeed > mMaxLinearSpeed) 
	    	currentLinearSpeed = mMaxLinearSpeed;
	    double diffAngle = atan2(dy, dx);
	    this->mX += currentLinearSpeed * std::cos(diffAngle);
	    this->mY += currentLinearSpeed * std::sin(diffAngle);

	    // update angle
	    diffAngle = getRotateAngle(dx, dy);
	    if (mAngularPrecision < std::abs(diffAngle)) {
	        mCurrentAngle = diffAngle;
	        while (mCurrentAngle > M_PI) 
	        	mCurrentAngle -= 2 * M_PI;
	        while (mCurrentAngle < -M_PI) 
	        	mCurrentAngle += 2 * M_PI;
	    }
	}

	double AbstractMoveableRobot::getRotateAngle(double dx, double dy)
	{
	    if (dy == 0) 
	    	return (dx < 0 ? M_PI : 0.0);
	    else {
	        if (dx == 0) 
	        	return (dy < 0 ? -M_PI/2 : M_PI/2);
	        double angleCos = dx / std::sqrt(dx * dx + dy * dy);
	        return (dy < 0 ? -std::acos(angleCos) : std::acos(angleCos));
	    }
	    return 0.0;
	}
	
}