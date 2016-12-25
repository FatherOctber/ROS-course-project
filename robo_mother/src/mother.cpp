	#include "mother.h"
	#include <gazebo_msgs/ModelState.h>
	#include <tf/transform_broadcaster.h>
	#include <tf/transform_listener.h>

	Mother::Mother(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z)
		:AbstractRobot(node, model_path, name, x, y, z)
	{
		//todo
	}
	
	Mother::~Mother()
	{
		//todo
	}
	
	void Mother::robotCallback(const robo_mother::command::ConstPtr& message)
	{
		if(message->is_online) {
			this->state = GlobalState::Online;
		} else {
			this->state = GlobalState::Offline;
		}
	}

	void Mother::controlRobots()
	{
		//todo robot control logic
	}
	
	void Mother::execute()
	{
		tf::TransformBroadcaster broadcaster;
		tf::Transform motherTransform;

		ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
		ros::Publisher succeedPublisher = node.advertise<robo_mother::command>("/command", 1000);
		ros::Subscriber succeedSupscriber = node.subscribe("/command", 10, &Mother::robotCallback, this);
		sleep(1.0);

		gazebo_msgs::ModelState robotState;
		robotState.model_name = this->getName();
		robotState.pose.position.x = this->getX();
		robotState.pose.position.y = this->getY();
		robotState.pose.orientation.x = 0.0;
		robotState.pose.orientation.y = 0.0;
		robotState.pose.orientation.z = 0.0;
		robotState.pose.orientation.w = 1.0;

		motherTransform.setOrigin(tf::Vector3(this->getX(), this->getX(), 0.0));
		motherTransform.setRotation(tf::Quaternion(0,0,0,1));
		broadcaster.sendTransform(tf::StampedTransform(motherTransform, ros::Time::now(), "world", this->getName()));
		ros::Rate rate(30);
		ROS_INFO("Start %s\n", this->getName().c_str());

		while ((state != GlobalState::Offline) && ros::ok())
		{
			if (state == GlobalState::Idle) {
				ROS_INFO("Mother start to command");
				state = GlobalState::Online;
			} 
			ros::spinOnce();
            motherTransform.setOrigin(tf::Vector3(this->getX(), this->getY(), 0.0));
			motherTransform.setRotation(tf::Quaternion(0,0,0,1));
			broadcaster.sendTransform(tf::StampedTransform(motherTransform, ros::Time::now(), "world", this->getName()));
        	gazeboPublisher.publish(robotState);
					
			rate.sleep();
		}
		ROS_INFO("Finish %s\n", this->getName().c_str());

	}
	
	
	