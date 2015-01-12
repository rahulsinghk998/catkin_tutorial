#include <iostream>
#include <ros/ros.h>
#include <catkin_tutorial/serial.h>
#include <string>

int main(int agrc, char** argv)
{
	Serial::serial testport(B9600,0,0,"/dev/ttyUSB2");
	testport.SerialConnect();
	while(1){
		//testport.SerialWrite("R");
		usleep(100);
		std::cout<<"READ DATA:  "<<testport.SerialRead()<<std::endl;
	}
return 0;
}
