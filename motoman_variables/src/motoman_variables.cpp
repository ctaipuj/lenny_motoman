#include"../include/motoman_variables/motoman_variables.h"

motoman_variables::motoman_variables(){
	address=0;
	value=0;
	write_client=n.serviceClient<motoman_msgs::WriteSingleIO>("write_single_io");
	read_client=n.serviceClient<motoman_msgs::ReadSingleIO>("read_single_io");
}

void motoman_variables::turnOn(int variable){
	address=variable;
	value=1;
	if(validateAddress())
	call_writter();
}
	
void motoman_variables::turnOff(int variable){
	address=variable;
	value=0;
	if(validateAddress())
	call_writter();
}

int motoman_variables::status(int variable){
	address=variable;
	if(validateAddress())
	return call_reader();
}

void motoman_variables::setAddressValue(int variable, int val){
	address=variable;
	value=val;
	if(variable>25010 && variable<=25147 && variable!=25012 && variable!=25014 && variable!=25016){
		ROS_WARN("You are attempting to set a value to an unused/undefined network input #%i",variable);
		call_writter();
	}else if(variable==25010 || variable==25012 || variable==25014 || variable==25016){
		call_writter();
	}
	else if(address>=0 && address<=6)
		ROS_WARN("Shortcuts 0-6 are only available via turnOn, turnOff or status methods.");
	else
		ROS_ERROR("ROS is not allowed to edit address #%i. Only network inputs #25XXX",variable);
}

int motoman_variables::getAddressValue(int variable){
	address=variable;
	if(variable>25010 && variable<=25147 && variable!=25012 && variable!=25014 && variable!=25016){
		ROS_WARN("You are attempting to read an unused/undefined network input #%i",variable);
		return call_reader();
	}else if(variable==25010 || variable==25012 || variable==25014 || variable==25016){
		return call_reader();
	}
	else if(address>=0 && address<=6)
		ROS_WARN("Shortcuts 0-6 are only available via turnOn, turnOff or status methods.");
	else
		ROS_ERROR("ROS is not allowed to access address #%i. Only network inputs #25XXX",variable);
	return -1;
}




void motoman_variables::call_writter(){
	write_srv.request.address=address;
	write_srv.request.value=value;
	if (write_client.call(write_srv))
		ROS_INFO("Address #%i has been set to %i",address,value);
	else
		ROS_ERROR("Failed to call service writeIO");
}

int motoman_variables::call_reader(){
	read_srv.request.address=address;
	if (read_client.call(read_srv)){
		ROS_INFO("Address #%i is currently set to %i",address,read_srv.response.value);
		return read_srv.response.value;
	}
	else{
		ROS_ERROR("Failed to call service readIO");
		return -1;
	}
}
	
bool motoman_variables::validateAddress(){
	if(address==0|| address==2 || address==4 || address==6){
		switch(address){
			case 0:
				address=25010;
				break;
			case 2:
				address=25012;
				break;
			case 4:
				address=25014;
				break;
			case 6:
				address=25016;
				break;
		}
		return true;
	}else if(address==25010 || address==25012 || address==25014 || address==25016)
		return true;
	else{
		ROS_ERROR("Invalid address #%i using this method. If address is valid use setAddressValue and getAddressValue methods",address);
		return false;
	}
}
