/******************************************************************************************************/
/*		      HELPFUL SITE: http://jbohren.com/articles/modular-ros-packages/                 */
/******************************************************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <sstream>
#include <std_msgs/UInt32.h>
#include <catkin_tutorial/serial.h>

#define BUFFER 200

uint32_t GetSensorData(Serial::serial *serialport);

uint32_t GetSensorData(Serial::serial *serialport){
	char sensorData[200]={'\0'};
	char* dataPtr;
	dataPtr=sensorData;
	int length=0;
	int i=0;
	while(length<255){
		length=serialport->SerialRead(dataPtr,BUFFER);	//REQUEST FOR DATA READ AND STORE LENGHT OF READ DATA
		printf("READ DATA: %s	LENGTH: %d \n",dataPtr, length);
		usleep(100);
		if(dataPtr[length-1]==' '){			//BREAKING CONDITION BETWEEN THE NEXT DATA I.E. A SPACEBAR
			std::cout<<sensorData<<std::endl;			
			break;
		}
		else{
			std::cout<<"TEST4: dataPtr[]  :: "<<dataPtr<<std::endl;
			dataPtr=dataPtr+length;
		}
		std::cout<<"1 :"<<dataPtr<<std::endl;

	}
	return atoi(sensorData);
}

int main(int argc, char** argv)
{

	ros::init(argc,argv,"serial_publisher_node");
	ros::NodeHandle nh;
	ros::Publisher publish_data=nh.advertise<std_msgs::UInt32>("IMU_DATA",1000);
	ros::Rate frequency(100);
	Serial::serial serialport(B9600,0,1,"/dev/ttyUSB1");
	serialport.SerialConnect();
	int i=0;
	while(ros::ok()){
		std::cout<<"COUNT:"<<i++<<std::endl;
		std_msgs::UInt32 publish;
		publish.data=GetSensorData(&serialport);
		publish_data.publish(publish);
		ros::spinOnce();
		frequency.sleep();
	}
serialport.SerialDisconnect();
return 0;
}
		 






























//fcntl(fd_serialport, F_SETFL, 0);         /* causes read to block until new characters are present */
//fcntl(fd_serialport, F_SETFL, FNDELAY);   /*it causes read to return immediately */



/*
	//Function returns the number of bytes to be read i.e. present in the buffer register.
	int Tserial::bytesToRead()
	{
    		int bytes=0;
    		ioctl(fd_serialport, FIONREAD, &bytes);
    		if(v==true) {
    	    		printf("\nbytesToRead %d ",bytes);
    		}
    		return bytes;
	}

	void Tserial::clear()
	{
    		tcflush(fd_serialport, TCIFLUSH);
    		tcflush(fd_serialport, TCOFLUSH);
	}


*/
