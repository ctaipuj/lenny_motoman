#include "../include/robotiq_85_control/gripper_ur_control.h" // use <robotiq_85_control/gripper_ur_control.h> in your node include, when working in your own package

int main(int argc, char* argv[]){

	ros::init(argc, argv, "gripper_test"); // init ROS nodes
	ros::NodeHandle n; 
	GripperUR g; //create an object of GripperUR class
	g.setCheckpointAddress(15); //not required
	g.init(); //inits gripper
	g.close(); //closes gripper
	g.moveto(50); //moves gripper to a position 0-255
	g.setTimeOut(10); 
	g.moveto(300); //valid invalid
	g.setSpeed(0); //0-255
	g.moveto(3);
	g.setForce(257); //valid invalid 0-255
	g.moveto(100);
	g.open();
	g.setSpeed(300); //valid invalid 0-255
	g.moveto(255);
	g.setSpeed(100);
	g.setForce(0);
	g.moveto(0);
	
	ros::Duration(1.0).sleep();
	
	return 0;
}
