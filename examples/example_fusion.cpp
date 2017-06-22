#include <ros/ros.h>
#include "cnbiros_navigation/SimpleFusion.hpp"

typedef cnbiros::navigation::SimpleFusion SimpleFusion;

int main(int argc, char** argv) {

	std::string topic = "/ros_robotino/infrared/";

	ros::init(argc, argv, "navigation_fusion");
	ros::NodeHandle node("~");
	ros::Rate r(10);

	SimpleFusion fusion(&node, "fusion");

	fusion.Add(topic, SimpleFusion::AsPointCloud); 

	fusion.Start();

	return 0;

}
