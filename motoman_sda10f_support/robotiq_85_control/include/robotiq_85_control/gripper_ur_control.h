#ifndef GRIPPER_UR_CONTROL
#define GRIPPER_UR_CONTROL

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>
#include <sstream>

/** @brief This class controls the robotiq 85 gripper through UR3 controller via sockets.
	
	This class uses rostopics URDriver/URScript, and URDriver/IOstates and rosservice setIO to control robotiq 85 model gripper. This class publishes commands in ur script format to control the robotiq gripper. This class requires that the robotiq's URcaps is installed in the controller and that the URDriver/URScript in published. The execution verification is done through a configurable digital output named checkpoint this is why the class verifies the IO states.
	ur_driver, ur_msgs are comonents required.
	@author Barrero Lizarazo, Nicolas
	@date September 2018
	*/ 

class GripperUR{
	
	private:
		ros::NodeHandle n;
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::ServiceClient client;
		
		std_msgs::String msg_out;
		ur_msgs::IOStates msg_in;
		ur_msgs::SetIO srv;
		
		const std::string HEADER="def gripper_action():\n socket_close(\"G\")\n sync()\n socket_open(\"127.0.0.1\",63352,\"G\")\n sync()\n";
		std::string FEEDBACK;//const std::string FEEDBACK=" set_configurable_digital_out(7,True)\n"; Now you can change the configurable output.
		const std::string ENDING=" socket_close(\"G\")\nend\n";
		 
		void reset();
	
		bool wait();
			
		void publisher();
		
		void updater();
		
		void writeFeedback(int id);
		
		std::string writeSpeed();
		
		std::string writeForce();
		
		std::string writePose(int pose);
		
		std::string writeTimeOut();
		
		std::string writeWhile(int pose, int t);
		
		std::string writeWhileOpen();
		
		std::string writeWhileClose();
		
		bool flag;
		bool init_ok;
		int pinID;
		int speed;
		int force;
		int pose;
		int tolerance;
		int time_out;
		
		std::string command;
		std::stringstream aux;
	
	public:
	/** The constructor initializes the subscriptor and publisher functions. Advertices the URdriver/URScript topic. Initialices variables.
	*/
		GripperUR();
	/** This is the regular Callback from a ros node, this function updates the checkpoint data used as a feedback from the UR controller.
		@param msg is the message type the nodes subscribes to: const ur_msgs::IOStates::ConstPtr&
	*/
		
		void gripper85_statusCallback(const ur_msgs::IOStates::ConstPtr& msg_in);
	/** The init function must be executed at first. It resets the gripper and executes the initialization routine. This function overrides other configured parameters previously defined. Once the init routine is done the gripper will move.
	*/
		void init();
	/** This function opens the gripper.
	*/		
		void open();
	/** This function closes the gripper.
	*/		
		void close();
	/** This function moves the gripper to a given position between 0-255. It executes the trayectory and waits until the goal is reached. If an object is detected the gripper stops the movement, should be called after init()
		@param goal 0-255 where 0 is totally open and 255 totally closed
	*/
		void moveto(int goal);
	/**This function sets gripper's speed
		@param int speed 0-255 where 255 is max speed (default 255)
	*/	
		void setSpeed(int s=255);
	/**This function sets gripper's force
		@param int force 0-255 where 255 is max force (default 0)
	*/	
		void setForce(int f=0);
	/**This function sets gripper's pose tolerance +/- (default 0)
		@param int tolerance en 0-255 units.
	*/		
		void setPoseTolerance(int t=0);
	/**This function sets controller time out (default 8s)
		@param int waiting time in seconds
	*/	
		void setTimeOut(int t=8);
	/**This function sets checkpoint address used for feedback. Only digital addresses are avaible: from 2x00 to 2x07. (default 2x07)
		@param int Address ID from 8->2x00 to 15->2x07
	*/		
		void setCheckpointAddress(int address=15);
	/**Gets the current gripper's speed
		@return int 0-255 see setSpeed()
	*/	
		int getSpeed();
	/**Gets the current gripper's force
		@return int 0-255 see setForce()
	*/	
		int getForce();
	/**Gets the current gripper's pose tolerance
		@return int 0-255 see setPoseTolerance()
	*/	
		int getPoseTolerance();
	/**Gets the current waiting time 
		@return int seconds see setTimeOut()
	*/	
		int getTimeOut();
	/**Gets the current output address used as checkpoint 
		@return int from 8->2x00 to 15->2x07 see setCheckpointAddress()
	*/	
		int getCheckpointAddress();		
};
#endif
