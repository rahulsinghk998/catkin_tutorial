#include <iostream>
#include <ros/ros.h>
#include <catkin_tutorial/serial.h>
#include <string>
#include <stdio.h>

#define BUFFER 255

int main(int agrc, char** argv)
{
	std::string serialRead;
	char data[BUFFER];
	int length;
	Serial::serial testport(B9600,0,0,"/dev/ttyUSB1");
	testport.SerialConnect();
	while(1){
		//testport.SerialWrite("R");
		//usleep(100);
		//Serial::serial testport(B9600,0,0,"/dev/ttyUSB0");
		//testport.SerialConnect();
		//serialRead=testport.SerialRead();
		//printf("DATA READ : %s\n",serialRead.c_str());
		//std::cout<<"READ DATA:  "<<data<<std::endl;
		//testport.SerialDisconnect();
		length=testport.SerialRead(data,BUFFER);
		printf("READ DATA: %s	LENGTH: %d \n",data, length);
		usleep(100);
		
	}
return 0;
}
