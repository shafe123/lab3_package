/* ---------------------------------------------------------------------------

This is just some Basic "boilerplate" code. I don't know more about this
project right now so I might as well get started.

-Naomi Hourihane, Sunday 9/29
------------------------------------------------------------------------------*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
//These are some of the same includes that were in my source code for lab 2.
//Idk what else to put for now


//some global variables can go here

float hand_posX = 0;
float hand_posY = 0;
float hand_posZ = 0;

float base_posX = 0;
float base_posY = 0;
float base_posZ = 0;

//Idk what global variables we are going to need so I'm just guessing.

//Put some callback functions here.

void HelloWorldCallback(){
	ROS_INFO("Hello World! This is a callback!\n");
}


int main(int argc, char **argv){
	ROS_INFO("Main node started!\n");

	ros::Rate loop_rate(1);//The loop rate is 1 hz
	while (ros::ok()){
		HelloWorldCallback;
	}
	return 0;
}
