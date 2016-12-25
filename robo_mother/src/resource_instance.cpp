#include "environment.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "resource_robot");
    ros::NodeHandle node;

	if (argc == 4) {
    	AbstractRobot* resourceRobot = new Resource(node, argv[1],
    	 	ros::this_node::getName(),
    	 	(double) atof(argv[2]),
    	 	(double) atof(argv[3]),
    	 	(double) atof(argv[4]));
		resourceRobot->spawn();
    	resourceRobot->execute();
	} else {
		ROS_ERROR("Unconsist parametres count");
	}

    return 0;
}
