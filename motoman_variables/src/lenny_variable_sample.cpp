#include<ros/ros.h>
#include "../include/motoman_variables/motoman_variables.h" // if you are using your own CMakeLists you can include <motoman_variables/motoman_variables.h>

int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "testing_to_lenny_i_o");
	ros::NodeHandle n;
	motoman_variables my_variable;
	my_variable.turnOn(25010);
	ros::Duration(5).sleep(); //for fun
	my_variable.turnOff(0);
	ros::Duration(5).sleep();
	my_variable.turnOn(25011);
	ros::Duration(5).sleep();
	my_variable.setAddressValue(25010,1);
	my_variable.setAddressValue(25020,1);
	ros::Duration(5).sleep();
	my_variable.setAddressValue(30010,1);
	ros::Duration(5).sleep();
	my_variable.turnOn(25012);
	my_variable.status(25012);
	ros::Duration(5).sleep();
	my_variable.getAddressValue(25020);
	my_variable.status(25020);
	my_variable.getAddressValue(0);
	ros::Duration(5).sleep();
	my_variable.getAddressValue(25010);
	my_variable.turnOff(0);
	ros::Duration(5).sleep();
	my_variable.setAddressValue(25020,0);
	ros::Duration(5).sleep();
	my_variable.turnOff(25012);
	
//	return 0;
}
