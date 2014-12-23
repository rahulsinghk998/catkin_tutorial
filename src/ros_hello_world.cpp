#include <ros/ros.h>
#include <iostream>
#include <string>

int main(int argc,char** argv)
{
	int i,j;
	for(i=0; i<5; i++){
		for(j=0; j<5; j++){
			std::cout<<"["<<i<<"]"<<"["<<j<<"]"<<"\t";
		}
		std::cout<<std::endl;
	}
return 0;
}
