#include "mother.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mother_robot");
    ros::NodeHandle node;

	if (argc == 5) {
    	AbstractRobot* mother = new Mother(node, argv[1],
    	 	ros::this_node::getName(),
    	 	(double) atof(argv[2]),
    	 	(double) atof(argv[3]),
    	 	(double) atof(argv[4]));
		mother->spawn();
    	mother->execute();
	} else {
		ROS_ERROR("Unconsistent parametres count");
	}

    return 0;
}
