	#include "scout.h"
	#include <gazebo_msgs/ModelState.h>
	#include <tf/transform_broadcaster.h>
	#include <tf/transform_listener.h>
	#include <random>

	Scout::Scout(ros::NodeHandle& node, const std::string& model_path, const std::string& name)
		: AbstractMoveableRobot(node, model_path, name)
	{
	}

	Scout::Scout(ros::NodeHandle& node, const std::string& model_path, const std::string& name, double x, double y, double z)
		:AbstractMoveableRobot(node, model_path, name, x, y, z)
	{
	}
	
    void Scout::explore(double atX, double atY)
	{
		//todo explore
	}
	
	Scout::~Scout()
	{
	}

	void Scout::robotCallback(const robo_mother::command::ConstPtr& message)
	{
		if(message->is_online) {
			this->state = GlobalState::Online;
		} else {
			this->state = GlobalState::Offline;
		}
	}

	void Scout::execute()
	{
		tf::TransformBroadcaster broadcaster;
		tf::Transform scoutTransform;
		
		std::random_device rd;
    	std::default_random_engine engine(rd());
    	std::uniform_real_distribution<> uniform_dist(-6.0, 6.0);
    	double goalX = uniform_dist(engine);
        double goalY = uniform_dist(engine);

		ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
		ros::Publisher succeedPublisher = node.advertise<robo_mother::command>("/command", 1000);
		ros::Subscriber succeedSupscriber = node.subscribe("/command", 10, &Scout::robotCallback, this);
		sleep(1.0);

		gazebo_msgs::ModelState robotState;
		robotState.model_name = this->getName();
		robotState.pose.position.x = this->getX();
		robotState.pose.position.y = this->getY();
		robotState.pose.orientation.x = 0.0;
		robotState.pose.orientation.y = 0.0;
		robotState.pose.orientation.z = 0.0;
		robotState.pose.orientation.w = 1.0;

		scoutTransform.setOrigin(tf::Vector3(this->getX(), this->getX(), 0.0));
		scoutTransform.setRotation(tf::Quaternion(0,0,0,1));
		broadcaster.sendTransform(tf::StampedTransform(scoutTransform, ros::Time::now(), "world", this->getName()));
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
				if (distance <= getDistancePrecision()) {
                    goalX = uniform_dist(engine);
                    goalY = uniform_dist(engine);
                } else {
                    move(dx, dy, distance);
                    robotState.pose.position.x = this->getX();
                	robotState.pose.position.y = this->getY();
                	robotState.pose.orientation.z = sin(this->getCurrentAngle() / 2);
                	robotState.pose.orientation.w = cos(this->getCurrentAngle() / 2);
                }
			}
			scoutTransform.setOrigin(tf::Vector3(this->getX(), this->getY(), 0.0));
			scoutTransform.setRotation(tf::Quaternion(0,0,0,1));
			broadcaster.sendTransform(tf::StampedTransform(scoutTransform, ros::Time::now(), "world", this->getName()));
			gazeboPublisher.publish(robotState);
				
			rate.sleep();

		}
		ROS_INFO("Finish %s\n", this->getName().c_str());

	}
