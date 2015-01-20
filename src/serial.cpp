/******************************************************************************************************/
/*		   HELPFUL SITE: http://tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html             */
/******************************************************************************************************/

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

#define BUFFER_SIZE 200
#define MISSING_VALUE 200
        
//#define MODEMDEVICE "/dev/ttyUSB2"
#define _POSIX_SOURCE 1 				//POSIX compliant source 

namespace Serial
{
	serial::serial(long int set_baud, int set_parity, int set_stop_bit,std::string set_port)
	{
		portIsConnected=false;
		baudrate=set_baud;
		parity=set_parity;
		stopBit=set_stop_bit;
		serialport=set_port;
		dataBits=8;
	}

	void serial::InitPort()
	{
		switch(baudrate){
			case 1200: 
				newtio.c_cflag |= B1200;
				break;
			case 2400:
				newtio.c_cflag |= B2400;
				break;
			case 4800:
				newtio.c_cflag |= B4800;
				break;
			case 9600:					//GIVES 1.17 KBPS
				newtio.c_cflag |= B9600;
				break;			
			case 19200:
				newtio.c_cflag |= B19200;
				break;
			case 38400:
				newtio.c_cflag |= B38400;
				break;
			case 57600:
				newtio.c_cflag |= B57600;
				break;
			case 115200:					//GIVES 125 KBPS
				newtio.c_cflag |= B115200;
				break;
			case 230400:					//GIVES 250 KBPS
				newtio.c_cflag |= B230400;
				break;
			case 460800:					//GIVES 0.5 MBPS
				newtio.c_cflag |= B460800;
				break;
			case 921600:					//GIVES 1 MBPS
				newtio.c_cflag |= B921600;
				break;
			default: 
				newtio.c_cflag |= B9600;
				break;
		 }
		 switch(parity){
			case 0:
				newtio.c_cflag &= ~PARENB;		//CLEAR PARITY ENABLE=>NO PARITY
				break;
			case 1:
				newtio.c_cflag |= PARENB;		//ENABLE PARITY
				newtio.c_cflag |= PARODD;		//ODD PARITY
				break;
			case 2:
				newtio.c_cflag |= PARENB;
				newtio.c_cflag |=~PARODD;		//NO ODD PARITY=>EVEN PARITY
				break;			
			default:
				newtio.c_cflag |= ~PARENB;		//DEFAULT=>NO PARITY
				break;
		 }
		 switch(stopBit){
			case 0:
				newtio.c_cflag &= ~CSTOPB;		//NO TWO STOP BITS=>ONE STOP BIT
				break;
			case 1:	
				newtio.c_cflag |=CSTOPB;		//TWO STOP BITS
				break;
			default:
				newtio.c_cflag &= ~CSTOPB;
				break;
		 }
		 switch(dataBits){
			case 7:
				newtio.c_cflag |= CS7;
				break;			
			case 8:
				newtio.c_cflag |= CS8;
				break;
			default:
				newtio.c_cflag |= CS8;
		 }
	         //newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
		 newtio.c_cflag |= CRTSCTS | CLOCAL | CREAD;
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
		handle = open(serialport.c_str(), O_RDWR | O_NOCTTY );	//Serial port init//Open port in readwrite 
	        if (handle<0) {					//If unsuccessful <fd> is <-1>
			perror(serialport.c_str()); 			//For <perror> function see at the bottom
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
	
	//ERROR :: The very first data is having error during the return.
	//std::string serial::SerialRead(void)
	int serial::SerialRead(char* serialData,int buffer)
	{
		//char serialData[255]={'\0'};
		//printf("SERIAL STRING CHECK : %s \n",serialData);
		//std::string serialData;
		//char* serialData;
		//strLength=read(handle,serialData.c_str(),255);
		//printf("SERIAL CHECK : %s 	SERIAL LENGTH:%d\n",serialData.c_str(),strLength);
		//printf("SECOND SERIAL CHECK :%c     SERIAL LENGTH:%d \n",serialData[0],strLength);
		//printf("SERIAL CHECK :%s     SERIAL LENGTH:%d \n",serialData,strLength);
		return read(handle,serialData,buffer);	
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


























//fcntl(fd_serialport, F_SETFL, 0);         /* causes read to block until new characters are present */
//fcntl(fd_serialport, F_SETFL, FNDELAY);   /* it causes read to return immediately */


/*
THE BAUDRATE CAN BE CHANGED BY USING THE FOLLOWING FUNCTION;
	n =  cfsetispeed(&terminalAttributes,B1200);
        n += cfsetospeed(&terminalAttributes,B1200);
*/

/*
**The C library function void perror(const char *str) prints a descriptive error message to stderr. First the string str is printed followed by a colon then a space.

void perror(const char *str)
	=>Parameter:	str-- This is the C string containing a custom message to be printed before the error message itself.
	=>Return: 	Doesn't returns anything.
	=>Link:		http://www.tutorialspoint.com/c_standard_library/c_function_perror.htm

*/
