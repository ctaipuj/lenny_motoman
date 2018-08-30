#ifndef MOTOMAN_VARIABLES
#define MOTOMAN_VARIABLES
//ros
#include <ros/ros.h> 
//Controller required services
#include <motoman_msgs/WriteSingleIO.h> 
#include <motoman_msgs/ReadSingleIO.h>

class motoman_variables{
	
	private:
		ros::NodeHandle n;
		motoman_msgs::WriteSingleIO write_srv;
		motoman_msgs::ReadSingleIO read_srv;
		ros::ServiceClient write_client;
		ros::ServiceClient read_client;
		
		int address;
		int value;
		
		void call_writter();
		int call_reader();
		bool validateAddress();
	
	public:
		motoman_variables();
		
		void turnOn(int variable);
			
		void turnOff(int variable);
		
		int status(int variable);
		
		void setAddressValue(int variable, int val);
		
		int getAddressValue(int variable);
};
#endif
