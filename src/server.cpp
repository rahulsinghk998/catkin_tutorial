#include <ros/ros.h>
#include <catkin_tutorial/Multiply.h>

bool MultiplyCallBack(catkin_tutorial::Multiply::Request &req, catkin_tutorial::Multiply::Response &res)
{
	res.product=(req.num1)*(req.num2);
	ROS_INFO("request: x=%ld, y=%ld",(long int)req.num1,(long int)req.num2);
	ROS_INFO("sending back the response: [%ld]",(long int)res.product);
	return true;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"Multiply_server_node");
	ros::NodeHandle node;
	ros::ServiceServer service=node.advertiseService("Multiply_2_num",MultiplyCallBack);
	ROS_INFO("Server Initialised and Ready to Multiply");
	while(ros::ok()){
		ros::spinOnce();
	}
return 0;
}

