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
		
	void Drone::move(double dx, double dy, double dz)
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
		ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
		ros::Publisher succeedPublisher = node.advertise<robo_mother::command>("/command", 1000);
		ros::Subscriber succeedSupscriber = node.subscribe("/command", 10, &Drone::robotCallback, this);
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
				
			rate.sleep();

		}
		ROS_INFO("Finish %s\n", this->getName().c_str());

	}