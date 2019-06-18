#include "../include/robotiq_85_control/gripper_ur_control.h"

GripperUR::GripperUR(){
	pub=n.advertise<std_msgs::String>("/ur_driver/URScript",5);
	sub=n.subscribe("/ur_driver/io_states",10, &GripperUR::gripper85_statusCallback,this);
	client=n.serviceClient<ur_msgs::SetIO>("/ur_driver/set_io");
	ros::Duration(1.0).sleep();
	setCheckpointAddress();
	setTimeOut();
	setPoseTolerance();
	setSpeed();
	setForce();
	flag=0;
	init_ok=0;
	command="";
}

void GripperUR::gripper85_statusCallback(const ur_msgs::IOStates::ConstPtr& msg_in){
		
	flag=msg_in->digital_out_states[pinID].state;
}

void GripperUR::init(){
	reset();
	ROS_WARN("gripper initialization");
	command=HEADER;
	command+=	" socket_set_var(\"ACT\",1,\"G\")\n";
	command+= " sync()\n";
	command+= " t=0\n";
	command+= " while (socket_get_var(\"STA\",\"G\")!=3 and t<25):\n";
	command+= "  sync()\n";
	command+= "  sleep(0.25)\n";
	command+= "  t=t+1\n";
	command+= " end\n";
	command+= " if(t>=25):\n";
	command+= "  popup(\"Something went wrong with gripper activation! TIME OUT\")\n";
	command+= "  halt\n";
	command+= " end\n";
	command+= " socket_set_var(\"GTO\",1,\"G\")\n";
	command+= " sync()\n";
	command+= " socket_set_var(\"SPE\",255,\"G\")\n";
	command+= " sync()\n";
	command+= " socket_set_var(\"FOR\",0,\"G\")\n";
	command+= " sync()\n";
	//command+= " sleep(0.5)\n";
	command+= " socket_set_var(\"POS\",0,\"G\")\n";
	command+= " sync()\n";
	command+= " t=0\n";
	command+= " while (socket_get_var(\"POS\",\"G\")>3 and t<25):\n";
	command+= "  sync()\n";
	command+= "  sleep(0.25)\n";
	command+= "  t=t+1\n";
	command+= " end\n";
	command+= " if(t>=25):\n";
	command+= "  popup(\"Something went wrong with gripper activation! TIME OUT\")\n";
	command+= "  halt\n";
	command+= " end\n";
	//command+= " sync()\n";
	command+= FEEDBACK;
	command+= ENDING;
	publisher();
	if(wait()){
		ROS_INFO("INIT OK");
		init_ok=1;
	}
	updater();
}

void GripperUR::open(){
	if(!init_ok){
		ROS_WARN("Activate gripper first");
		goto end;
	}
	command=HEADER;
	command+= writeSpeed();
	command+= " sync()\n";
	command+= writeForce();
	command+= " sync()\n";
	command+= " socket_set_var(\"POS\",0,\"G\")\n";
	command+= " sync()\n";
	command+= " t=0\n";
	command+= writeWhileOpen();
	command+= "  sync()\n";
	command+= "  sleep(0.25)\n";
	command+= "  t=t+1\n";
	command+= " end\n";
	command+= writeTimeOut();
	command+= "  popup(\"TIME OUT\")\n";
	command+= "  halt\n";
	command+= " end\n";
	command+= FEEDBACK;
	command+= ENDING;
	publisher();
	if(wait())
		ROS_INFO("Gripper succesfully opened");
	updater();
	
	end:;
}

void GripperUR::close(){
	if(!init_ok){
		ROS_WARN("Activate gripper first");
		goto end;
	}
	command=HEADER;
	command+= writeSpeed();
	command+= " sync()\n";
	command+= writeForce();
	command+= " sync()\n";
	command+= " socket_set_var(\"POS\",255,\"G\")\n";
	command+= " sync()\n";
	command+= " t=0\n";
	command+= writeWhileClose();
	command+= "  sync()\n";
	command+= "  sleep(0.25)\n";
	command+= "  t=t+1\n";
	command+= " end\n";
	command+= writeTimeOut();
	command+= "  popup(\"TIME OUT\")\n";
	command+= "  halt\n";
	command+= " end\n";
	command+= FEEDBACK;
	command+= ENDING;
	publisher();
	if(wait())
		ROS_INFO("Gripper succesfully closed");
	updater();
	
	end:;
}

void GripperUR::moveto(int goal){
	if(!init_ok){
		ROS_WARN("Activate gripper first");
		goto end;
	}
	command=HEADER;
	command+= writeSpeed();
	command+= " sync()\n";
	command+= writeForce();
	command+= " sync()\n";
	command+= writePose(goal);
	command+= " sync()\n";
	command+= " t=0\n";
	command+= writeWhile(goal,tolerance);
	command+= "  sync()\n";
	command+= "  sleep(0.25)\n";
	command+= "  t=t+1\n";
	command+= " end\n";
	command+= writeTimeOut();
	command+= "  popup(\"TIME OUT\")\n";
	command+= "  halt\n";
	command+= " end\n";
	command+= FEEDBACK;
	command+= ENDING;
	publisher();
	if(wait()){
		if(goal>255)
			ROS_INFO("Gripper pose goal: %i reached",255);
		else if(goal<0)
			ROS_INFO("Gripper pose goal: %i reached",0);
		else
			ROS_INFO("Gripper pose goal: %i reached",goal);
	}
	updater();
	
	end:;
}

void GripperUR::setSpeed(int s){
	if(s>255)
		speed=255;
	else if(s<0)
		speed=0;
	else
		speed=s;
	ROS_INFO("Current gripper speed->%i",speed);
}

void GripperUR::setForce(int f){
	if(f>255)
		force=255;
	else if(f<0)
		force=0;
	else
		force=f;
	ROS_INFO("Current gripper force->%i",force);
}

void GripperUR::setPoseTolerance(int t){
	tolerance=t;
	ROS_INFO("Pose Tolerance->%i",tolerance);
}

void GripperUR::setTimeOut(int t){
	if(t>0)
		time_out=t/0.25;
	else
		time_out=8;
	ROS_INFO("Gripper's time out set to %i seconds",t);
}

void GripperUR::setCheckpointAddress(int address){
	if(address>15 || address<8){
		srv.request.fun=1;
		srv.request.pin=15;
		srv.request.state=0;
		pinID=15;
		writeFeedback(pinID);
		ROS_WARN("Invalid Address, setting default address 2x07 instead.\n Only configurable outputs IDs 8 to 15 available");
	}
	else{
		srv.request.fun=1;
		srv.request.pin=address;
		srv.request.state=0;
		pinID=address;
		writeFeedback(pinID);
		if(pinID!=15)
			ROS_INFO("Checkpoint address changed to %i", address);
		else
			ROS_INFO("Default Checkpoint Address 2x07");
	}
}

int GripperUR::getSpeed(){
	return speed;
}

int GripperUR::getForce(){
	return force;
}

int GripperUR::getPoseTolerance(){
	return tolerance;
}
		
int GripperUR::getTimeOut(){
	return time_out*0.25;
}
		
int GripperUR::getCheckpointAddress(){
	return pinID;
}




void GripperUR::reset(){
	command= HEADER;
	command+= " socket_set_var(\"ACT\",0,\"G\")\n";
	command+= " sync()\n";
	/*command+= " socket_set_var(\"GTO\",0,\"G\")\n"; Works better without this.
	command+= " sync()\n";
	command+= " socket_set_var(\"SPE\",0,\"G\")\n";
	command+= " sync()\n";
	command+= " socket_set_var(\"FOR\",0,\"G\")\n";
	command+= " sync()\n";*/
	//command+= " socket_set_var(\"POS\",0,\"G\")\n";
	//command+= " socket_set_var(\"PRE\",0,\"G\")\n";
	command+= " sync()\n";
	command+= " sleep(0.5)\n";
	command+= FEEDBACK;
	command+= ENDING;
	publisher();
	if(wait())
		init_ok=0;
	updater();
}

bool GripperUR::wait(){
	int time=0;
	while(!flag && ros::ok() && time<time_out){
		ros::spinOnce();
		ros::Duration(0.25).sleep();
		time++;
	}
	if(time>=time_out){
		ROS_ERROR("Command TIME OUT");
		return 0;
	}
	return 1;
}
			
void GripperUR::publisher(){
	msg_out.data=command;
	pub.publish(msg_out);
	command="";
}
		
void GripperUR::updater(){
	if (!client.call(srv))
		ROS_ERROR("Unable to update controller data");
	ros::Duration(0.5).sleep();
	ros::spinOnce();
}	

void GripperUR::writeFeedback(int id){
	aux.str("");
	aux<<" set_configurable_digital_out("<<id-8<<",True)\n";
	FEEDBACK= aux.str();
}

std::string GripperUR::writeSpeed(){
	aux.str("");
	aux<<" socket_set_var(\"SPE\","<<speed<<",\"G\")\n";
	return aux.str();
}

std::string GripperUR::writeForce(){
	aux.str("");
	aux<<" socket_set_var(\"FOR\","<<force<<",\"G\")\n";
	return aux.str();
}
		
std::string GripperUR::writePose(int pose){
	aux.str("");
	if(pose>255){
		aux<<" socket_set_var(\"POS\","<<255<<",\"G\")\n";
		ROS_WARN("Setting max valid position goal 255");
	}
	else if(pose<0){
		aux<<" socket_set_var(\"POS\","<<0<<",\"G\")\n";
		ROS_WARN("Setting min valid position goal 0");
	}
	else
		aux<<" socket_set_var(\"POS\","<<pose<<",\"G\")\n";
	return aux.str();
}
		
std::string GripperUR::writeTimeOut(){
	aux.str("");
	aux<<" if(t>="<<time_out<<"):\n";
	return aux.str();
}

std::string GripperUR::writeWhileOpen(){
	aux.str("");
	aux<<" while (socket_get_var(\"POS\",\"G\")>3 and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
	return aux.str();
}
		
std::string GripperUR::writeWhileClose(){
	aux.str("");
	aux<<" while (socket_get_var(\"POS\",\"G\")<227 and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
	return aux.str();
}
		
std::string GripperUR::writeWhile(int pose, int t){
	aux.str("");
	if(pose>230)
		aux<<" while (socket_get_var(\"POS\",\"G\")<"<<227-t<<" and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
	else if(pose<3)
		aux<<" while (socket_get_var(\"POS\",\"G\")>"<<3+t<<" and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
	else
		aux<<" while ((socket_get_var(\"POS\",\"G\")<"<<pose-t<<" or socket_get_var(\"POS\",\"G\")>"<<pose+t<<") and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
	return aux.str();
}
