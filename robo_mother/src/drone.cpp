	#include "drone.h"
	#include <gazebo_msgs/ModelState.h>
	#include <tf/transform_broadcaster.h>
	#include <tf/transform_listener.h>
	#include <random>

	Drone::Drone(ros::NodeHandle& node, const std::string& model_path, const std::string& name)
		: AbstractMoveableRobot(node, model_path, name)
	{
	}

	Drone::Drone(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z)
		:AbstractMoveableRobot(node, model_path, name, x, y, z)
	{
	}
	
    void Drone::mine(double atX, double atY,  double atZ)
	{
		//todo mining logic
	}
	
	Drone::~Drone()
	{
	}

	void Drone::robotCallback(const robo_mother::command::ConstPtr& message)
	{
		if(message->is_online) {
			this->state = GlobalState::Online;
		} else {
			this->state = GlobalState::Offline;
		}
	}

	void Drone::motheranswerCallback(const robo_mother::robotstate::ConstPtr& message)
	{
		std::cout << getName() << ": GOT IT" <<std::endl;
		int target = message->state;
		if (id == target) {
			targetItem = new AllStateItem();
			targetItem->id = message->id;
			targetItem->robottype = message->robottype;
			targetItem->x = message->x;
			targetItem->y = message->y;
			targetItem->state = message->state;
			isMotherAnswered = true;

			std::cout << getName() << ": HAS RESOURCE" <<std::endl;
		} else {
			std::cout << getName() << ": NOT MINE" <<std::endl;
		}
	}

	void Drone::decreaseFromResCallback(const robo_mother::robotstate::ConstPtr& message)
	{
		int target = message->id;
		if (id == target) {
			int res = message->state;
			takenRes = &res;
		} 
	}
	void Drone::motherIncrementCallback(const robo_mother::robotstate::ConstPtr& message)
	{
		int target = message->id;
		if (id == target) {
			isMotherAnswered = true;
		}
	}
	
	void Drone::execute()
	{
		tf::TransformBroadcaster broadcaster;
		tf::Transform resourceTransform;
		//for test only
		std::random_device rd;
    	std::default_random_engine engine(rd());
    	std::uniform_real_distribution<> uniform_dist(-6.0, 6.0);
    	double goalX = uniform_dist(engine);
        double goalY = uniform_dist(engine);

		//spawnRobot(node, robot.getName(), "/home/nmishnev/.gazebo/models/pioneer2dx/yellow_model.sdf", robot.getX(), robot.getY(), 0.0);
		gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
		succeedPublisher = node.advertise<robo_mother::command>("/command", 1000);
		robotstatePublisher = node.advertise<robo_mother::robotstate>("/robotstate", 1000);
		resourceDecreasePublisher = node.advertise<robo_mother::robotstate>("/resourcedecrease", 1000);
		resourceWantedPublisher = node.advertise<robo_mother::robotstate>("/resourcewanted", 1000);
		succeedSupscriber = node.subscribe("/command", 1000, &Drone::robotCallback, this);
		sleep(2.0);

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
				state = GlobalState::Online;
			} else {
				ros::spinOnce();
				double dx = goalX - this->getX();
                double dy = goalY - this->getY();
                double distance = std::sqrt(dx * dx + dy * dy);
                move(dx, dy, distance);
                robotState.pose.position.x = this->getX();
                robotState.pose.position.y = this->getY();
                robotState.pose.orientation.z = sin(this->getCurrentAngle() / 2);
                robotState.pose.orientation.w = cos(this->getCurrentAngle() / 2);
			}
			resourceTransform.setOrigin(tf::Vector3(this->getX(), this->getY(), 0.0));
			resourceTransform.setRotation(tf::Quaternion(0,0,0,1));
			broadcaster.sendTransform(tf::StampedTransform(resourceTransform, ros::Time::now(), "world", this->getName()));
			gazeboPublisher.publish(robotState);
				
			robo_mother::robotstate rstate;
			rstate.id = 0;
			rstate.robottype = 0;
			rstate.x = this->getX();
			rstate.y = this->getY();
			rstate.state = 0;
			robotstatePublisher.publish(rstate);

			rate.sleep();

			waitForAnswerFromMother();
		}
		ROS_INFO("Finish %s\n", this->getName().c_str());

	}

	void Drone::waitForAnswerFromMother()
	{
		std::cout << getName() << ": READY TO MOVE" <<std::endl;
		ros::Rate rate(30);
		wannaAnswerSubscriber = node.subscribe("/answerFromMother", 1000, &Drone::motheranswerCallback, this);
		sleep(2.0);
		ros::spinOnce();
		rate.sleep();

		tf::TransformBroadcaster broadcaster;
		tf::Transform resourceTransform;
		isMotherAnswered = false;
		targetItem = NULL;

		robo_mother::robotstate rstate;
		rstate.id = id;
		rstate.robottype = 0;
		rstate.x = this->getX();
		rstate.y = this->getY();
		rstate.state = 0;
		resourceWantedPublisher.publish(rstate);

		gazebo_msgs::ModelState robotState;
		robotState.model_name = this->getName();
		robotState.pose.position.x = this->getX();
		robotState.pose.position.y = this->getY();
		robotState.pose.orientation.x = 0.0;
		robotState.pose.orientation.y = 0.0;
		robotState.pose.orientation.z = 0.0;
		robotState.pose.orientation.w = 1.0;

		while (!isMotherAnswered && ros::ok())
		{
			ros::spinOnce();
                robotState.pose.position.x = this->getX();
                robotState.pose.position.y = this->getY();
                robotState.pose.orientation.z = sin(this->getCurrentAngle() / 2);
                robotState.pose.orientation.w = cos(this->getCurrentAngle() / 2);
			resourceTransform.setOrigin(tf::Vector3(this->getX(), this->getY(), 0.0));
			resourceTransform.setRotation(tf::Quaternion(0,0,0,1));
			broadcaster.sendTransform(tf::StampedTransform(resourceTransform, ros::Time::now(), "world", this->getName()));
			gazeboPublisher.publish(robotState);
				
			robo_mother::robotstate rstate;
			rstate.id = 0;
			rstate.robottype = 0;
			rstate.x = this->getX();
			rstate.y = this->getY();
			rstate.state = 0;
			robotstatePublisher.publish(rstate);
			rate.sleep();
		}
		wannaAnswerSubscriber.shutdown();
		if (isMotherAnswered)
			moveToTarget();
		else
			std::cout << getName() << ": EXIT FROM READY TO MOVE" <<std::endl;
	}

	void Drone::moveToTarget() {
		std::cout << getName() << ": MOVE TO TARGET" <<std::endl;
		tf::TransformBroadcaster broadcaster;
		tf::Transform resourceTransform;
		ros::Rate rate(30);

		gazebo_msgs::ModelState robotState;
		robotState.model_name = this->getName();
		robotState.pose.position.x = this->getX();
		robotState.pose.position.y = this->getY();
		robotState.pose.orientation.x = 0.0;
		robotState.pose.orientation.y = 0.0;
		robotState.pose.orientation.z = 0.0;
		robotState.pose.orientation.w = 1.0;
		double goalX = targetItem->x;
		double goalY = targetItem->y; 
		while (ros::ok())
		{
			ros::spinOnce();

				double dx = goalX - this->getX();
                double dy = goalY - this->getY();
                double distance = std::sqrt(dx * dx + dy * dy);
                move(dx, dy, distance);
                robotState.pose.position.x = this->getX();
                robotState.pose.position.y = this->getY();
                robotState.pose.orientation.z = sin(this->getCurrentAngle() / 2);
                robotState.pose.orientation.w = cos(this->getCurrentAngle() / 2);

			resourceTransform.setOrigin(tf::Vector3(this->getX(), this->getY(), 0.0));
			resourceTransform.setRotation(tf::Quaternion(0,0,0,1));
			broadcaster.sendTransform(tf::StampedTransform(resourceTransform, ros::Time::now(), "world", this->getName()));
			gazeboPublisher.publish(robotState);

			robo_mother::robotstate rstate;
			rstate.id = 0;
			rstate.robottype = 0;
			rstate.x = this->getX();
			rstate.y = this->getY();
			rstate.state = 0;
			robotstatePublisher.publish(rstate);

			rate.sleep();

			if (abs(distance) <= 0.001) {
				takeResource();
				return;
			}
		}
	}

	void Drone::takeResource() {
		std::cout << getName() << ": TAKE RESOURCE" <<std::endl;
		ros::Rate rate(30);
		takenRes = NULL;
		wannaDecreaseSubscriber = node.subscribe("/decreaseFromRes", 1000, &Drone::decreaseFromResCallback, this);
		sleep(2.0);
		ros::spinOnce();
		rate.sleep();

		robo_mother::robotstate rstate;
		rstate.id = targetItem->id;
		rstate.robottype = 0;
		rstate.x = this->getX();
		rstate.y = this->getY();
		rstate.state = id;
		resourceDecreasePublisher.publish(rstate);

		while (takenRes == NULL && ros::ok())
		{
			ros::spinOnce();
			rate.sleep();
		}
		
		wannaDecreaseSubscriber.shutdown();
		if (takenRes != NULL) {
			std::cout << getName() << ": DESREASED RESOURCE" <<std::endl;
			moveToHome();
		}

	}

	void Drone::moveToHome() {
		std::cout << getName() << ": MOVE TO TARGET" <<std::endl;
		tf::TransformBroadcaster broadcaster;
		tf::Transform resourceTransform;
		ros::Rate rate(30);

		gazebo_msgs::ModelState robotState;
		robotState.model_name = this->getName();
		robotState.pose.position.x = this->getX();
		robotState.pose.position.y = this->getY();
		robotState.pose.orientation.x = 0.0;
		robotState.pose.orientation.y = 0.0;
		robotState.pose.orientation.z = 0.0;
		robotState.pose.orientation.w = 1.0;
		double goalX = 0.0;
		double goalY = 0.0; 
		while (ros::ok())
		{
			ros::spinOnce();

				double dx = goalX - this->getX();
                double dy = goalY - this->getY();
                double distance = std::sqrt(dx * dx + dy * dy);
                move(dx, dy, distance);
                robotState.pose.position.x = this->getX();
                robotState.pose.position.y = this->getY();
                robotState.pose.orientation.z = sin(this->getCurrentAngle() / 2);
                robotState.pose.orientation.w = cos(this->getCurrentAngle() / 2);

			resourceTransform.setOrigin(tf::Vector3(this->getX(), this->getY(), 0.0));
			resourceTransform.setRotation(tf::Quaternion(0,0,0,1));
			broadcaster.sendTransform(tf::StampedTransform(resourceTransform, ros::Time::now(), "world", this->getName()));
			gazeboPublisher.publish(robotState);

			robo_mother::robotstate rstate;
			rstate.id = 0;
			rstate.robottype = 0;
			rstate.x = this->getX();
			rstate.y = this->getY();
			rstate.state = 0;
			robotstatePublisher.publish(rstate);

			rate.sleep();

			if (abs(distance) <= 0.001) {
				giveResource();
				return;
			}
		}
	}

	void Drone::giveResource() {
		if (*takenRes < 0) {
			std::cout << getName() << ": NOTHING THERE :(" <<std::endl;
			return;
		}
		ros::Rate rate(30);
		wannaAnswerSubscriber = node.subscribe("/answerFromMother", 1000, &Drone::motherIncrementCallback, this);
		sleep(2.0);
		ros::spinOnce();
		rate.sleep();

		isMotherAnswered = false;
		robo_mother::robotstate rstate;
		rstate.id = id;
		rstate.robottype = 1;
		rstate.x = this->getX();
		rstate.y = this->getY();
		rstate.state = *takenRes;
		resourceWantedPublisher.publish(rstate);

		while (!isMotherAnswered && ros::ok())
		{
			ros::spinOnce();
			rate.sleep();
		}

		wannaAnswerSubscriber.shutdown();
		std::cout << getName() << ": MOTHER IS HAPPY :)" <<std::endl;
	}
