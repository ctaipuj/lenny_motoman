#include<ros/ros.h>
#include "../include/motoman_variables/motoman_variables.h" // if you are using your own CMakeLists you can include <motoman_variables/motoman_variables.h>

int main(int argc, char** argv)
{

	ros::init(argc, argv, "motoman_variables");

	motoman_variables my_variable;;

	ros::Rate loop_rate(1000);
	while(ros::ok()){
	
		ros::spinOnce();
		loop_rate.sleep();


	}

	ros::waitForShutdown();
	return 0;

} 
