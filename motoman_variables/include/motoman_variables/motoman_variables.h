#ifndef MOTOMAN_VARIABLES
#define MOTOMAN_VARIABLES
//ros
#include <ros/ros.h> 
//Controller required services
#include <motoman_msgs/WriteSingleIO.h> 
#include <motoman_msgs/ReadSingleIO.h>

#define I0 25010
#define I2 25012
#define I4 25014
#define I6 25016

/** @brief This class is used to set, reset and edit network inputs of the FS100 robot controller.
	
	ROS only can write/read network inputs #25xxx. This class uses the ROS services WriteSingleIO and ReadSingleIO to access and edit controller's data. To generate an external output it is necessary to edit the ladder program of the controler in order to activate the #30xxx outputs. Currently the 25010, 25012, 25014 and 25016 network inputs are connected to external outputs 30030, 30032, 30034 and 30036 respectively. This class accepts the shorcuts 0, 2, 4, 6 too. This class allso defines the inputs as I0, I2, I3, I4; I6. You can read/write any other network input however currently they are no related to any external output.
	@author Barrero Lizarazo, Nicolas
	@date September 2018
	*/

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
	/** The constructor initializes two service clients one from WriteSingleIO service and the other from ReadSingleIO. 
		This service uses the messages from the edited version of the motoman_msgs package
		*/
		motoman_variables();
	/** Sets a network input to true/1. You can pass either the address direction, shortcuts (0, 1, 2, 4, 6) or the class definitions I0, I2, I4, I6.
		 @param variable an int that represents the address ID
		NOTE: if you are trying to set a network input that is not connected to an external output use setAdressValue() method instead.
		*/ 
		
		void turnOn(int variable);
	/** Sets a network input to false/0. You can pass either the address direction, shortcuts (0, 1, 2, 4, 6) or the class definitions I0, I2, I4, I6.
		 @param variable an int that represents the address ID
		NOTE: if you are trying to set a network input that is not connected to an external output use setAdressValue() method instead.
		*/
			
		void turnOff(int variable);
	/** Gets a network input state. You can pass either the address direction, shortcuts (0, 1, 2, 4, 6) or the class definitions I0, I2, I4, I6.
		 @param variable an int that represents the address ID
		 @return 0 the address is reset 1 it is set
		NOTE: if you are trying to get the status of a network input that is not connected to an external output use getAdressValue() method instead.
		*/
		int status(int variable);
	/** Sets a network input to false/0 or true/1. You only can pass the address direction.
		 @param variable an int that represents the address ID
		 @param val 1 for set, 0 for reset
		NOTE: if you are trying to modify a "favorite" network input, which are the ones connected to an external output, it is recommended to use turnOn() turnOff() methods instead.
		*/		
		void setAddressValue(int variable, int val);
	/** Gets a network input current value. You only can pass the address direction.
		 @param variable an int that represents the address ID
		 @return 0 the address is reset 1 it is set
		NOTE: if you are trying to get the status of a "favorite" network input, which are the ones connected to an external output, it is recommended to use status() method instead.
		*/		
		int getAddressValue(int variable);
};
#endif
