#include <ros/ros.h>
#include <serial.h>
//#include <error.h>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"serial_data_publisher_node");
	
