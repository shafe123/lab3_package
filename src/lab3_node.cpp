
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>//These are some of the same includes that were in my source code for lab 2.

#include "std_srvs/Trigger.h"

//some global variables can go here


//some callback functions go here

void HelloWorldCallback(){
	ROS_INFO("Hello world! This is a callback!\n");
	//Idk where this is printing to but it's getting called correctly
}


int main(int argc, char **argv){
	ROS_INFO("Main node started!\n");
	ros::init(argc, argv, "lab3_package_node");//does this have to match name of node in CMakeLists?
	ros::NodeHandle handle; //THE NODEHANDLE IS "handle" !!!
	

		
	ros::ServiceClient begin_client = handle.serviceClient<std_srvs::Trigger>("Service Name");
	//Rename the service name??
	//Declares a variable of type ros::ServiceClient called begin_client
	
	std_srvs::Trigger begin_comp;
	//Declares a variable of type std_srvs::Trigger called begin_comp
	
	begin_client.call(begin_comp); //Call the service
	
	ROS_WARN("Competition service returned failure: %s \n", begin_comp.response.message.c_str());


	ros::Rate loop_rate(1);//loop rate is 1 hz
	while (ros::ok()){
		HelloWorldCallback;
		ROS_INFO("Calling callback function!\n");
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
