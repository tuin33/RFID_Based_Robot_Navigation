#include "RFIDHandler_nav.h"
#include <string>
#include <sstream>
#include "time.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <std_msgs/String.h>
#include <utility>
#include <thread>
#include <chrono>
#include <RFID_Based_Robot_Navigation/RFIDdata.h>


using namespace std;

void keyboardDetect(bool *flag)
{
	int ch;
	while(1)
	{
		ch = getchar();
		if (ch == 65)
		{
			*flag = 1;
			break;
		}
	}
}

int main(int argc, char **argv)
{
	// setup ROS node
	ros::init(argc, argv, "reader1");
	
	// initialize Impinj reader
	char *pReaderHostName;

	pReaderHostName = "192.168.0.105"; //115.156.142.223

	CMyApplication myApp;
	myApp.m_Verbose = 1;
	myApp.m_channelIndex = 1;

	int rc;

	ROS_INFO("Ready to read ...");
	ros::Duration(2).sleep();

	ros::Rate loop_rate(10);

	rc = myApp.run(pReaderHostName);

	printf("Reader has been shut down.\n");

	return 0;
}
