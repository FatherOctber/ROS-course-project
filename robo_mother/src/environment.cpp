	#include "environment.h"
	#include <gazebo_msgs/ModelState.h>
	#include <tf/transform_broadcaster.h>
	#include <tf/transform_listener.h>

	StandartBlock::StandartBlock(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z)
		:AbstractRobot(node, model_path, name, x, y, z)
	{
		//todo
	}
	
	StandartBlock::~StandartBlock()
	{
		//todo
	}
	
	void StandartBlock::robotCallback(const robo_mother::command::ConstPtr& message)
	{
		if(message->is_online) {
			this->state = GlobalState::Online;
		} else {
			this->state = GlobalState::Offline;
		}
	}
	
	void StandartBlock::execute()
	{
		tf::TransformBroadcaster broadcaster;
		tf::Transform blockTransform;

		//spawnRobot(node, robot.getName(), "/home/nmishnev/.gazebo/models/pioneer2dx/yellow_model.sdf", robot.getX(), robot.getY(), 0.0);
		ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
		ros::Publisher succeedPublisher = node.advertise<robo_mother::command>("/command", 1000);
		ros::Subscriber succeedSupscriber = node.subscribe("/command", 10, &StandartBlock::robotCallback, this);
		sleep(1.0);

		gazebo_msgs::ModelState robotState;
		robotState.model_name = this->getName();
		robotState.pose.position.x = this->getX();
		robotState.pose.position.y = this->getY();
		robotState.pose.orientation.x = 0.0;
		robotState.pose.orientation.y = 0.0;
		robotState.pose.orientation.z = 0.0;
		robotState.pose.orientation.w = 1.0;

		blockTransform.setOrigin(tf::Vector3(this->getX(), this->getX(), 0.0));
		blockTransform.setRotation(tf::Quaternion(0,0,0,1));
		broadcaster.sendTransform(tf::StampedTransform(blockTransform, ros::Time::now(), "world", this->getName()));
		ros::Rate rate(30);
		ROS_INFO("Start %s\n", this->getName().c_str());

		while ((state != GlobalState::Offline) && ros::ok())
		{
			if (state == GlobalState::Idle) {
				ROS_INFO("Stone start to be the stone");
				state = GlobalState::Online;
			} 
			ros::spinOnce();
            blockTransform.setOrigin(tf::Vector3(this->getX(), this->getY(), 0.0));
			blockTransform.setRotation(tf::Quaternion(0,0,0,1));
			broadcaster.sendTransform(tf::StampedTransform(blockTransform, ros::Time::now(), "world", this->getName()));
        	gazeboPublisher.publish(robotState);
					
			rate.sleep();
		}
		ROS_INFO("Finish %s\n", this->getName().c_str());

	}
	
	Resource::Resource(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z, int size)
		:AbstractRobot(node, model_path, name, x, y, z)
	{
		resources = 5;
		rotationAngle = 0.0;
	}
	
	Resource::~Resource()
	{
		//todo
	}
	
	int Resource::getSize()
	{
		return resources;
	}
	
	int Resource::getRandomPart()
	{
		int part = 20; //todo randomize this
		resources = resources - part;
		return part;
	}

	void Resource::robotCallback(const robo_mother::command::ConstPtr& message)
	{
		if(message->is_online) {
			this->state = GlobalState::Online;
		} else {
			this->state = GlobalState::Offline;
		}
	}

	void Resource::decreaseCallback(const robo_mother::robotstate::ConstPtr& message) {
		int target = message->id;
		if (target == id) {
			std::cout << getName() << ": DECREASED( " << resources <<std::endl;
			resources -= 1;

				robo_mother::robotstate rstate;
				rstate.id = message->state;
				rstate.robottype = 1;
				rstate.x = getX();
				rstate.y = getY();
				rstate.state = resources;
				ros::spinOnce();
				decreasePublisher.publish(rstate);
				ros::spinOnce();
				return;
		}
	}

	void Resource::execute()
	{
		tf::TransformBroadcaster broadcaster;
		tf::Transform resourceTransform;

		ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
		ros::Publisher succeedPublisher = node.advertise<robo_mother::command>("/command", 1000);
		decreasePublisher = node.advertise<robo_mother::robotstate>("/decreaseFromRes", 1000);
		ros::Subscriber succeedSupscriber = node.subscribe("/command", 1000, &Resource::robotCallback, this);
		decreaseSubscriber = node.subscribe("/resourcedecrease", 1000, &Resource::decreaseCallback, this);
		robotstatePublisher = node.advertise<robo_mother::robotstate>("/robotstate", 1000);
		sleep(1.0);

		gazebo_msgs::ModelState robotState;
		robotState.model_name = this->getName();
		robotState.pose.position.x = this->getX();
		robotState.pose.position.y = this->getY();
		robotState.pose.orientation.x = 0.0;
		robotState.pose.orientation.y = 0.0;
		robotState.pose.orientation.z = 0.0;
		robotState.pose.orientation.w = 1.0;

		resourceTransform.setOrigin(tf::Vector3(this->getX(), this->getX(), 0.0));
		resourceTransform.setRotation(tf::Quaternion(0,0,0,1));
		broadcaster.sendTransform(tf::StampedTransform(resourceTransform, ros::Time::now(), "world", this->getName()));
		ros::Rate rate(30);
		ROS_INFO("Start %s\n", this->getName().c_str());

		while ((state != GlobalState::Offline) && ros::ok())
		{
			if (state == GlobalState::Idle) {
				ros::spinOnce();
				ROS_INFO("Rosource start to be the resource");
				state = GlobalState::Online;
			} else {
				ros::spinOnce();
				if( getSize() >= 0 ) {
					robotState.pose.position.x = this->getX();
	                robotState.pose.position.y = this->getY();
	                robotState.pose.orientation.z = sin(this->rotationAngle / 2);
	                robotState.pose.orientation.w = cos(this->rotationAngle / 2);
	                if(this->rotationAngle >= 360) {
	                	this->rotationAngle = 0.0;
	                } else {
	                	this->rotationAngle++;
	                }
            	}
			}
			resourceTransform.setOrigin(tf::Vector3(this->getX(), this->getY(), 0.0));
			resourceTransform.setRotation(tf::Quaternion(0,0,0,1));
			broadcaster.sendTransform(tf::StampedTransform(resourceTransform, ros::Time::now(), "world", this->getName()));
			gazeboPublisher.publish(robotState);
				

			robo_mother::robotstate rstate;
			rstate.id = id;
			rstate.robottype = 1;
			rstate.x = this->getX();
			rstate.y = this->getY();
			rstate.state = resources;
			robotstatePublisher.publish(rstate);

			rate.sleep();

		}
		ROS_INFO("Finish %s\n", this->getName().c_str());

	}
	
	