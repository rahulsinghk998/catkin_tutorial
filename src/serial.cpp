#include <iostream>
#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sstream>
#include <unistd.h>
#include <catkin_tutorial/serial.h>
//#include <ErrorCodes.h>

#define BUFFER_SIZE 1024
#define MISSING_VALUE 1024

#define BAUDRATE B38400            
#define MODEMDEVICE "/dev/ttyUSB2"
#define _POSIX_SOURCE 1 				//POSIX compliant source 

namespace Serial
{
	serial::serial(long int set_baud, int set_parity, int set_stop_bit,std::string set_port)
	{
		portIsConnected=false;
		baudrate=set_baud;
		parity=set_parity;
		stopbit=set_stop_bit;
		serialport=set_port;
	}

	void serial::InitPort()
	{
	         newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
	         newtio.c_iflag = IGNPAR | ICRNL;
	         newtio.c_oflag = 0;
	         newtio.c_lflag = ICANON;
	         //initialize all control characters default values can be found in /usr/include/termios.h
	         newtio.c_cc[VINTR]    = 0;     // Ctrl-c  
	         newtio.c_cc[VQUIT]    = 0;     // Ctrl-\ 
	         newtio.c_cc[VERASE]   = 0;     // del 
	         newtio.c_cc[VKILL]    = 0;     // @ 
         	 newtio.c_cc[VEOF]     = 4;     // Ctrl-d 
	         newtio.c_cc[VTIME]    = 0;     // inter-character timer unused 
	         newtio.c_cc[VMIN]     = 1;     // blocking read until 1 character arrives 
	         newtio.c_cc[VSWTC]    = 0;     // '\0' 
	         newtio.c_cc[VSTART]   = 0;     // Ctrl-q  
	         newtio.c_cc[VSTOP]    = 0;     // Ctrl-s 
	         newtio.c_cc[VSUSP]    = 0;     // Ctrl-z 
	         newtio.c_cc[VEOL]     = 0;     // '\0' 
	         newtio.c_cc[VREPRINT] = 0;     // Ctrl-r 
	         newtio.c_cc[VDISCARD] = 0;     // Ctrl-u 
	         newtio.c_cc[VWERASE]  = 0;     // Ctrl-w 
	         newtio.c_cc[VLNEXT]   = 0;     // Ctrl-v 
	         newtio.c_cc[VEOL2]    = 0;     // '\0' 
	
	}
	
	int serial::SerialConnect()
	{
		handle = open(MODEMDEVICE, O_RDWR | O_NOCTTY );	//Serial port init//Open port in readwrite 
	        if (handle<0) {					//If unsuccessful <fd> is <-1>
			perror(MODEMDEVICE); 			//For <perror> function see at the bottom
			return 1;				//return(RQ_ERR_OPEN_PORT); 
		}
		else {
			std::cout<<"The Serial Port :"<<serialport<<"is connected"<<std::endl;		
			portIsConnected=true;
		}
		tcgetattr(handle,&oldtio);			//save current port setting in <oldtio>
		InitPort();
	        tcflush(handle,TCIFLUSH);			//Clean Modem Line
	        tcsetattr(handle,TCSANOW,&newtio);		//Activate Settings for PORT
	}

	char* serial::SerialRead(void)
	{
		char serialData[255];
		int strLength;
		strLength=read(handle,serialData,255);
		return serialData;
	}
	
	bool serial::SerialWrite(std::string serialSend)
	{
		if(!portIsConnected){
			std::cout<<"THE PORT IS NOT CONNECTED"<<std::endl;
			return 2;
		}
		int writeCount;
		writeCount=write(handle,serialSend.c_str(),serialSend.length());
		if(writeCount<0){
			std::cout<<"WRITE UNSUCESSFUL"<<std::endl;
			return 3;
		}
		usleep((25+serialSend.length())*100);
		return 4;
	}
	
	void serial::SerialDisconnect(void)
	{
		tcsetattr(handle,TCSANOW,&oldtio);			//Restore old settings of PORT
		close(handle);						//Close the opened PORT
		portIsConnected=false;
	}
	
	bool serial::SerialConnectCheck(void)
	{
		return portIsConnected;
	}

}


























































/*
**The C library function void perror(const char *str) prints a descriptive error message to stderr. First the string str is printed followed by a colon then a space.

void perror(const char *str)
	=>Parameter:	str-- This is the C string containing a custom message to be printed before the error message itself.
	=>Return: 	Doesn't returns anything.
	=>Link:		http://www.tutorialspoint.com/c_standard_library/c_function_perror.htm

*/
