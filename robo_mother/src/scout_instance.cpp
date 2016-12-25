#include "scout.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scout_robot");
    ros::NodeHandle node;

	if (argc == 5) {
    	AbstractRobot* scout = new Scout(node, argv[1],
    	 	ros::this_node::getName(),
    	 	(double) atof(argv[2]),
    	 	(double) atof(argv[3]),
    	 	(double) atof(argv[4]));
		scout->spawn();
    	scout->execute();
	} else {
		ROS_ERROR("Unconsistent parametres count");
	}

    return 0;
}
