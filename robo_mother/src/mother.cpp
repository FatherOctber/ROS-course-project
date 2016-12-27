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

	void Mother::robotstateCallback(const robo_mother::robotstate::ConstPtr& message)
	{
		AllStateItem rstate;
		rstate.id = message->id;
		rstate.robottype = message->robottype;
		rstate.x = message->x;
		rstate.y = message->y;
		rstate.state = message->state;
		allrobots.erase(rstate.id);
		allrobots.insert(std::make_pair(rstate.id, rstate));
		//std::cout<<message->id<<" "<<message->robottype<<" "<<message->x<<" "<<message->y<<" "<<message->state<<std::endl;
	}

	void Mother::resourcewantedCallback(const robo_mother::robotstate::ConstPtr& message) {
		if (message->robottype == 1) {
			resources++;
			std::cout << getName() << ": WE HAVE " << resources <<std::endl;
				robo_mother::robotstate rstate;
				rstate.id = message->id;
				rstate.robottype = 1;
				rstate.x = 0;
				rstate.y = 0;
				rstate.state = 0;
				ros::spinOnce();
				resourceFoundPublisher.publish(rstate);
				ros::spinOnce();
			return;
		}
		std::cout << getName() << ": RESOURCE FROM MOTHER" <<std::endl;
		int fromId = message->id;
		for (std::map<int, AllStateItem>::iterator it = allrobots.begin(); it != allrobots.end(); ++it) {
			AllStateItem stateitem = it->second;
			if (stateitem.robottype == 1 && stateitem.state > 0) {
				robo_mother::robotstate rstate;
				rstate.id = stateitem.id;
				rstate.robottype = 0;
				rstate.x = stateitem.x;
				rstate.y = stateitem.y;
				rstate.state = fromId;
				ros::spinOnce();
				resourceFoundPublisher.publish(rstate);
				ros::spinOnce();
				std::cout << getName() << ": SENDING RESOURCE FROM MOTHER" <<std::endl;
				return;
			}
		}

		std::cout << getName() << ": RESOURCE NOT FOUND" <<std::endl;
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
		resourceFoundPublisher = node.advertise<robo_mother::robotstate>("/answerFromMother", 1000);
		ros::Subscriber succeedSupscriber = node.subscribe("/command", 1000, &Mother::robotCallback, this);
		ros::Subscriber robotstateSupscriber = node.subscribe("/robotstate", 1000, &Mother::robotstateCallback, this);
		ros::Subscriber resourceWantedSubscriber = node.subscribe("/resourcewanted", 1000, &Mother::resourcewantedCallback, this);
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
	
	
	