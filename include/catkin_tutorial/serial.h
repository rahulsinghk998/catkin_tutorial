#ifndef Serial_serial_h
#define Serial_serial_h

#include <ros/ros.h>
#include <iostream>
#include <string>

namespace Serial
{
	class serial
	{
		public:			
			serial(long int set_baud, int set_parity, int set_stop_bit,string set_port);
			void InitPort(void);
			int SerialConnect();
			char* SerialRead(void);
			bool SerialWrite(char* serialSend,int dataCount);
			void SerialDisconnect(void);
			bool SerialConnectCheck(void);
		
		private:
			unsigned int baudrate;
			int parity;
			int stopbit;
			string serialport;
			bool portIsConnected;
			struct termios oldtio,newtio;
			int handle;
		protected:
			
	};

}

#endif
