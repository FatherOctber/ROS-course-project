#include "environment.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "block_robot");
    ros::NodeHandle node;

	if (argc == 5) {
    	AbstractRobot* blockRobot = new StandartBlock(node, argv[1],
    	 	ros::this_node::getName(),
    	 	(double) atof(argv[2]),
    	 	(double) atof(argv[3]),
    	 	(double) atof(argv[4]));
		blockRobot->spawn();
    	blockRobot->execute();

	} else {
		ROS_ERROR("Unconsistent parametres count");
	}

    return 0;
}
