#include <ros/ros.h>
#include <catkin_tutorial/Multiply.h>
#include <iostream>
#include <string>
#include <stdlib.h>


int main(int argc,char** argv)
{
	ros::init(argc,argv,"Multiply_client_node");
	if(argc!=3){	
		ROS_INFO("ussage: Multiply_client_node");
	}
	ros::NodeHandle node;
	ros::ServiceClient client=node.serviceClient<catkin_tutorial::Multiply>("Multiply_2_num");
	catkin_tutorial::Multiply srv;
	srv.request.num1=atoll(argv[1]);        	//<atoll> converts anything to long long int
	srv.request.num2=atoll(argv[2]);
	
	if(client.call(srv)){
		ROS_INFO("Success: The product is: %ld",(long int)srv.response.product);
	}
	else{
		ROS_INFO("FAILED to call the server to multiply the numbers");
		return 1;
	}
	
return 0;
}
	

//<atoi>   <==> convert string to integer
//<atol>   <==> convert string to long integer
//<strtoll <==> convert strint to long long interger
