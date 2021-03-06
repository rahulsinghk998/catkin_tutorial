#include <ros/ros.h>
#include <fstream>				//Header file for file handling i.e working with files
#include <sstream>
#include <iostream>				//Header file for input and ouput of data = cout $ cin
#include <stdio.h>
#include <std_msgs/UInt32.h>
#include <catkin_tutorial/serial.h>
#include <string>

void CallBackSubscriberNode(const std_msgs::UInt32::ConstPtr& msg)
{
	uint32_t sensorPublishedData;			//STORE THE SENSOR DATA VALUES
	std::ofstream fileobj("SensorData.txt",std::ios::out | std::ios::app);
	if(fileobj.is_open()!=true){
		std::cout<<"The FILE cannot be written"<<std::endl;
	}
	fileobj<<msg->data<<"\n";
	fileobj.close();

	//CODE TO ANALYSE THE DATA AND INTERPRETE ACCORDING TO OUR CONDITION
	//PIECE  OF CODE TO PLOT A GRAPH
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"serial_subscriber_node");
	ros::NodeHandle nh;
	ros::Subscriber subscriber_data=nh.subscribe("IMU_DATA",200,CallBackSubscriberNode);
	ros::spin();
return 0;
}




















//How to store data without the use of <append> mode of file using pointerst to the file object.
/*

void open(const char *filename, ios::openmode mode);

	Mode Flag	Description
	ios::app	Append mode. All output to that file to be appended to the end.
	ios::ate	Open a file for output and move the read/write control to the end of the file.
	ios::in		Open a file for reading.
	ios::out	Open a file for writing.
	ios::trunc	If file already exists, its contents will be truncated before opening the file.

	e.g.
	ofstream outfile;
	outfile.open("file.dat", ios::out | ios::trunc );

	fstream  afile;
	afile.open("file.dat", ios::out | ios::in );
*/

/*
POSITIIONING THE FILE POINTER LOCATION

	// position to the nth byte of fileObject (assumes ios::beg)
	fileObject.seekg( n );
	// position n bytes forward in fileObject
	fileObject.seekg( n, ios::cur );
	// position n bytes back from end of fileObject
	fileObject.seekg( n, ios::end );
	// position at end of fileObject
	fileObject.seekg( 0, ios::end );
*/

//**************************************************//
//FILE HANDLING//
//**************************************************//
/*
	ofstream FILE;				//Creating a file // In terms of C a 'file pointer'
	FILE.open("FILENAME.txt");		//Open the file or Create if not present
	FILE<<"Enter the text to be saved"<<endl; 
	FILE.close();				//Close the open file and delete the buffered data


	ofstream FILE("FILENAME.txt");		//Use of constructor to link the FILE
	FILE.is_open();				//To check if the object is associated with the file

*/

/*
	ifstream FILE("FILENAME.txt");		//Read the data from a document
	FILE>>stored_info1>>stored_info2;	//Returns NULL i.e. False boolian if EOF pointer reached.
	

*/
