#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc,char **argv)
{
//Used to initialize the node and name mapping //There are different version of <ros::init()> 
	ros::init(argc,argv,"talker");	


//Node access point for ros.It<constructor> will fully initialize the node and <deconstructor> will close down the node.
	ros::NodeHandle rah;


//<Advertize is the main function as it sets the topic name and type and bufferSize of the data that will be send on the node.
	ros::Publisher publish_data=rah.advertise<std_msgs::String>("chatter",100);


//It sets the frequency of operation of the node.
	ros::Rate frequency(10);
	int unique_count=0;
	while(ros::ok()){
		std_msgs::String data_send;


//stringstream is used for storing a bufferdata.Then it can be stored in a string.<NEED TO SEARCH A MORE>
		std::stringstream string_data;
		string_data<<"rahul"<<unique_count;
		ROS_INFO("%d\n",unique_count);
		data_send.data=string_data.str();

		ROS_INFO("%s",data_send.data.c_str());
		publish_data.publish(data_send);


//<SPINONCE> is used for callback function i.e. making them activated for callback when the next loop runs again.
		ros::spinOnce();


//<SLEEP> is used for making the loop go in sleep mode for the rest of remain time of the TimePeriod.
		frequency.sleep();
		unique_count++;
	}


return 0;
}















/*

<<http://docs.ros.org/diamondback/api/rostime/html/classros_1_1Rate.html>>

<<http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>>

Duration ros::Rate::cycleTime()  		//To get the actual Time period of the <while> execution loop from start to sleep

Duration ros::Rate::expectedCycleTime() 	//to get expected cycle time base on the frequency passed in constructor.

ros::Rate                          		//To set the desired frequency rate of the while loop.

void reset()					//sets the start time for the rate to now.

void sleep()					//sleep for the leftover time of execution loop.

*/
