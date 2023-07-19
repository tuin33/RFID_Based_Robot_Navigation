#include <ros/ros.h>
// #include "RFIDapi.h"
#include <iostream>
#include <thread>
#include <RFID_Based_Robot_Navigation/rfid_msgs.h>
#include <RFID_Based_Robot_Navigation/tag_msgs.h>
#include "RFIDHandler.h"

// CMyApplication CMyApplication::instance("192.168.0.105");

int main(int argc, char** argv){
    ros::init(argc, argv, "rfid_node_start");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<RFID_Based_Robot_Navigation::rfid_msgs>("/rfid_msgs", 1000);
    char *pReaderHostName = "192.168.0.105";

    // CMyApplication::instance = CMyApplication(pReaderHostName);
    CMyApplication myApp(pReaderHostName);
	myApp.m_Verbose = 1;

    std::thread t1(&CMyApplication::start, &myApp);

    myApp.init_keyboard();
    std::thread t2(&CMyApplication::keyboardThread, &myApp);

    ros::Rate rate(10);

    ros::Duration(2).sleep();

    std::cout << "----------------------" << std::endl;
    std::cout << "Press 'q' to quit and kill reader process." << std::endl;
    std::cout << "If directly press 'ctrl+c' to close this process." << std::endl;
    std::cout << "Please run stop node to kill reader process." << std::endl;
    std::cout << "----------------------" << std::endl;

    while(ros::ok() && myApp.isRunning){
        if(!myApp.dataVec.empty()){
            RFID_Based_Robot_Navigation::rfid_msgs msg;
            msg.Header.stamp = ros::Time::now();
            for (int i = 0; i < myApp.dataVec.size(); i++){
                RFID_Based_Robot_Navigation::tag_msgs tag_msg;
                tag_msg.epc = std::string(myApp.dataVec[i].epc);
                tag_msg.antID = myApp.dataVec[i].antennaId;
                tag_msg.phase = myApp.dataVec[i].phase;
                tag_msg.rssi = myApp.dataVec[i].rssi;
                tag_msg.timestamp_ros = myApp.dataVec[i].timestamp_ros;
                tag_msg.timestamp_pc = myApp.dataVec[i].timestamp_pc;

                msg.tag_array.push_back(tag_msg);
            }
            pub.publish(msg);
            myApp.dataVec.clear();
        }
        else{
            ROS_INFO("No data received.");
        }
        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "Reader has been shut down." << std::endl;


    // ROS_INFO("Ready to read ...");
    // ros::Duration(2).sleep();

    // int rc = myApp.run(pReaderHostName);
    
    // std::cout << "Reader has been shut down." << rc << std::endl;

    return 0;
}