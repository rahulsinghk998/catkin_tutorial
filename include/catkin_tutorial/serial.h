#ifndef Serial_serial_h
#define Serial_serial_h

#include <iostream>
#include <stdio.h>
#include <string>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sstream>
#include <unistd.h>

namespace Serial
{
	class serial
	{
		public:			
			serial(long int set_baud, int set_parity, int set_stop_bit,std::string set_port);
			void InitPort(void);
			int SerialConnect();
			//std::string SerialRead(void);
			int SerialRead(char* serialData, int buffer);
			bool SerialWrite(std::string serialSend);
			void SerialDisconnect(void);
			bool SerialConnectCheck(void);
		
		private:
			unsigned int baudrate;
			int parity;
			int stopBit;
			std::string serialport;
			bool portIsConnected;
			struct termios oldtio,newtio;
			int handle;
			int dataBits;
		protected:
			
	};

}

#endif
