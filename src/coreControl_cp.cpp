#include "ros/ros.h"
#include <ros/callback_queue.h>
#include "std_msgs/String.h"
#include <boost/thread.hpp>
#include <thread>
#include "RFID_Based_Robot_Navigation/RFIDdata.h"
#include "RFID_Based_Robot_Navigation/sendEnding.h"
#include "RFID_Based_Robot_Navigation/rfid_msgs.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <stdio.h>
#include <string>
#include "tf/transform_datatypes.h"
#include <ros/console.h>
#include "MainEntry.h"

class multiThreadListener
{

private:
    ros::NodeHandle n_main;
    ros::NodeHandle n_robot;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Publisher motion_publish;

public:
    OdomData odom;
    OdomData odom_save;
    Controller control;
    std::vector<double> robotX_recv;
    std::vector<double> robotY_recv;
    std::vector<double> robotW_recv;
    std::vector<double> robotTimeStamp_recv;

    std::vector<TagData> leftTagDataArray;
    std::vector<TagData> rightTagDataArray;

    // count iteration time

    multiThreadListener() {}

    ~multiThreadListener();

    void startRunning();

    void chatterCallback1(const nav_msgs::Odometry::ConstPtr &msg);
    void chatterCallback2(const RFID_Based_Robot_Navigation::rfid_msgs::ConstPtr &msg);
    void chatterCallback3(const RFID_Based_Robot_Navigation::sendEnding::ConstPtr &msg);

    void loopCalculate();

    void vel_publish();
};

multiThreadListener::~multiThreadListener()
{
    ROS_INFO("Quit core Control.");
     // Data transport
     char fileName1[200] = {0};
    std::ofstream data;
    sprintf(fileName1, "//home//tzq//data//test//nav//RFIDdata_left.txt");
    if (data.fail())
    {
        ROS_INFO("Open file 1 failed");
    }
    else
    {
        data.open(fileName1, std::ios::out);
        ROS_INFO("file 1 is open");
        int cnt1 = 0;
        for (int i = 0; i<leftTagDataArray.size(); i++)
        {
            data << ++cnt1 << " " << leftTagDataArray[i].epcBuffer << " "<< leftTagDataArray[i].AntennaID << " " << leftTagDataArray[i].phase << " " << leftTagDataArray[i].timestamp << endl;
        }
        data.close();
    }

    char fileName2[200] = {0};
    std::ofstream data1;
    sprintf(fileName2, "//home//tzq//data//test//nav//RFIDdata_right.txt");
    if (data1.fail())
    {
        ROS_INFO("Open file 2 failed");
    }
    else
    {
        data1.open(fileName1, std::ios::out);
        ROS_INFO("file 2 is open");
        int cnt2 = 0;
        for (int i = 0; i<rightTagDataArray.size(); i++)
        {
            data1 << ++cnt2 << " " << rightTagDataArray[i].epcBuffer << " "<< rightTagDataArray[i].AntennaID << " " << rightTagDataArray[i].phase << " " << rightTagDataArray[i].timestamp << endl;
        }
        data1.close();
    }

    char fileName3[200] = {0};
    std::ofstream Odom;
    sprintf(fileName3, "//home//tzq//data//test//nav//odom.txt");
    if (Odom.fail())
    {
        ROS_INFO("Open file 3 failed");
    }
    else
    {
        Odom.open(fileName3, std::ios::out);
        ROS_INFO("file 3 is open");
        //int cnt1 = 0;
        for (auto it1 = 0; it1 != odom_save.robot_x.size(); it1++)
        {
            Odom << std::fixed << odom_save.robot_x[it1] << " " << odom_save.robot_y[it1] << " " << odom_save.robot_th[it1] << " " << odom_save.robot_timestamp[it1] << endl;
            //cnt1++;
            // data1 << ++cnt1 << " " << (*it1).epc << " " << (*it1).channelIndex << " " << ((*it1).channelIndex - 1) * 0.25 + 920.625 << " " << (*it1).phase << " " << (*it1).rssi << endl;
        }
        Odom.close();
    }
}

// run spin
void multiThreadListener::startRunning()
{
    ros::CallbackQueue callback_queue_robot;
    n_robot.setCallbackQueue(&callback_queue_robot);
    sub1 = n_robot.subscribe<nav_msgs::Odometry>("/RosAria/pose", 20, &multiThreadListener::chatterCallback1, this);
    sub2 = n_main.subscribe("/rfid_msgs", 20, &multiThreadListener::chatterCallback2, this);
    // sub3 = n_main.subscribe("/Ending_msg", 20, &multiThreadListener::chatterCallback3, this);
    motion_publish = n_main.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 20);
    std::thread spinner_thread_robot([&callback_queue_robot]()
                                     {
            ros::SingleThreadedSpinner spinner_robot;
            spinner_robot.spin(&callback_queue_robot); });
    std::thread vel_pub(&multiThreadListener::vel_publish, this);
    std::thread loop_calculate(&multiThreadListener::loopCalculate, this);
    ros::spin();
    spinner_thread_robot.join();
}

// callback of robot
void multiThreadListener::chatterCallback1(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ROS_INFO("Receiving from robot.");

    // save robot's data
    robotTimeStamp_recv.push_back(msg->header.stamp.toSec());
    robotX_recv.push_back(msg->pose.pose.position.x);
    robotY_recv.push_back(msg->pose.pose.position.y);
    odom_save.robot_timestamp.push_back(msg->header.stamp.toSec());
    odom_save.robot_x.push_back(msg->pose.pose.position.x);
    odom_save.robot_y.push_back(msg->pose.pose.position.y);


    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double rollTemp = 0, pitchTemp = 0, yawTemp = 0;
    tf::Matrix3x3(quat).getRPY(rollTemp, pitchTemp, yawTemp);
    robotW_recv.push_back(yawTemp);
    odom_save.robot_th.push_back(yawTemp);
}

// call back of RFID
void multiThreadListener::chatterCallback2(const RFID_Based_Robot_Navigation::rfid_msgs::ConstPtr &msg)
{
    ROS_INFO("Receiving from reader.");

    // ROS_INFO("Receiving from tag: epc: %s, ant: %d", msg->epc.c_str(), msg->antID);

    TagData tagData;
    // std::cout << "Antenna " << msg->antID << " :"<<"TagDataArray is recieved!" << std::endl;
    // for (int i = 0; i < msg->phase.size(); i++)
    // {
    //     tagData.epcBuffer = msg->epc;
    //     tagData.AntennaID = msg->antID;
    //     tagData.phase = msg->phase[i];
    //     tagData.timestamp = msg->timestamp[i];
    //     if (msg->antID == LEFT_READER_ID)
    //     {
    //         leftTagDataArray.push_back(tagData);
    //         //cout<<"leftTagDataArray.size():"<<leftTagDataArray.size()<<endl;
    //     }
    //     else if (msg->antID == RIGHT_READER_ID)
    //     {
    //         rightTagDataArray.push_back(tagData);
    //         //cout<<"rightTagDataArray.size():"<<rightTagDataArray.size()<<endl;
    //     }
    // }

    for (auto it1 = msg->tag_array.begin(); it1 != msg->tag_array.end(); ++it1){
        if((*it1).epc == TARGET_TAG_EPC){
            for(int i = 0; i < (*it1).phase.size(); ++i){
                tagData.epcBuffer = (*it1).epc;
                tagData.AntennaID = (*it1).antID[i];
                tagData.phase = (*it1).phase[i];
                tagData.timestamp = (*it1).timestamp_ros[i];
                if ((*it1).antID[i] == LEFT_READER_ID)
                {
                    leftTagDataArray.push_back(tagData);
                    //cout<<"leftTagDataArray.size():"<<leftTagDataArray.size()<<endl;
                }
                else if ((*it1).antID[i] == RIGHT_READER_ID)
                {
                    rightTagDataArray.push_back(tagData);
                    //cout<<"rightTagDataArray.size():"<<rightTagDataArray.size()<<endl;
                }
            }
        }
            
    }
    
}

void multiThreadListener::loopCalculate(){
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if(control.iteration_count < 2000)
        {   
            if(leftTagDataArray.size()<=5 || rightTagDataArray.size()<=5)
                continue;
            tuple<double, double> motion = control.getMotion(&leftTagDataArray, &rightTagDataArray, odom_save, control.iteration_count);
            double linear_x = get<0>(motion);
            double angular = get<1>(motion);
            // Tempolimits einhalten
            if (linear_x > LINEAR_MAX_VEL)
            {
                linear_x = LINEAR_MAX_VEL;
            }
            else if (linear_x < -LINEAR_MAX_VEL)
            {
                linear_x = -LINEAR_MAX_VEL;
            }

            if (angular > ANGULAR_MAX_VEL)
            {
                angular = ANGULAR_MAX_VEL;
            }
            else if (angular < -ANGULAR_MAX_VEL)
            {
                angular = -ANGULAR_MAX_VEL;
            }

            control.vel_msg.linear.x = linear_x;
            control.vel_msg.angular.z = angular;
            ROS_INFO("[%0.2f m/s, %0.2f rad/s]", control.vel_msg.linear.x, control.vel_msg.angular.z);
            if(linear_x==0 && angular==0)
                break; 
            // leftTagDataArray.clear();
            // rightTagDataArray.clear();
            control.iteration_count++;
        }
        else
        {
            control.vel_msg.linear.x = 0.0;
            control.vel_msg.angular.z = 0.0;
            ROS_INFO("[%0.2f m/s, %0.2f rad/s]", control.vel_msg.linear.x, control.vel_msg.angular.z);
            break;
        }
        motion_publish.publish(control.vel_msg);
        loop_rate.sleep();
    }
     char fileName1[200] = {0};
    std::ofstream data1;
    // sprintf(fileName2, "//home//tzq//data//test//nav//RFIDdata_right.txt");
    sprintf(fileName1, "//home//tzq//catkin_ws//src//RFID_Based_Robot_Navigation//data//odom.txt");
    if (data1.fail())
    {
        cout << "File 1 is not open" << endl;
    }
    else
    {
        data1.open(fileName1, std::ios::out);
        for (int i = 0; i < control.iteration_count; i++)
        {
            data1 << std::fixed << control.robot_xt[i] << " " << control.robot_yt[i] << " " << control.robot_tht[i] << std::endl;
        }
        data1.close();
    }

    std::cout << "Finish writing data! 1" << std::endl;

    char fileName3[200] = {0};
    std::ofstream data3;
    // sprintf(fileName2, "//home//tzq//data//test//nav//RFIDdata_right.txt");
    sprintf(fileName3, "//home//tzq//catkin_ws//src//RFID_Based_Robot_Navigation//data//PF_point_center.txt");
    if (data3.fail())
    {
        cout << "File 3 is not open" << endl;
    }
    else
    {
        data3.open(fileName3, std::ios::out);
        for (int i = 0; i < control.iteration_count; i++)
        {
            data3 << std::fixed << control.PF_center_mean(0, i) << " " << control.PF_center_mean(1, i) << std::endl;
        }
        data3.close();
    }

    // char fileName2[200] = {0};
    // std::ofstream data2;
    // // sprintf(fileName2, "//home//tzq//data//test//nav//RFIDdata_right.txt");
    // sprintf(fileName2, "../experiment/result/PF_point_all_after.txt");
    // if (data2.fail())
    // {
    //     cout << "File 2 is not open" << endl;
    // }
    // else
    // {
    //     data2.open(fileName2, std::ios::out);
    //     for (int i = 0; i < 491; i++)
    //     {
    //         for (int j = 0; j < 1000; j++)
    //             data2 << std::fixed << control.PF_point_all_x_after[i](0, j) << " " << control.PF_point_all_y_after[i](0, j) << std::endl;
    //     }
    //     data2.close();
    // }

    // std::cout << "Finish writing data! 2" << std::endl;

    char fileName4[400] = {0};
    std::ofstream data4;
    // sprintf(fileName4, "//home//tzq//data//test//nav//RFIDdata_right.txt");
    sprintf(fileName4, "//home//tzq//catkin_ws//src//RFID_Based_Robot_Navigation//data//PF_point_all.txt");
    if (data4.fail())
    {
        cout << "File 4 is not open" << endl;
    }
    else
    {
        data4.open(fileName4, std::ios::out);
        for (int i = 0; i < control.iteration_count; i++)
        {
            for (int j = 0; j<1000; j++)
                data4 << std::fixed << control.PF_point_all_x[i](0, j) << " " << control.PF_point_all_y[i](0, j) << std::endl;
        }
        data4.close();
    }

    std::cout << "Finish writing data! 4" << std::endl;
}

// call back of stop flag
// void multiThreadListener::chatterCallback3(const RFID_Based_Robot_Navigation::sendEnding::ConstPtr &msg)
// {
//     ROS_INFO("Received ending info.");
//     char fileName1[200] = {0};
//     std::ofstream data;
//     sprintf(fileName1, "//home//tzq//data//test//nav//RFIDdata_left.txt");
//     if (data.fail())
//     {
//         ROS_INFO("Open file 1 failed");
//     }
//     else
//     {
//         data.open(fileName1, std::ios::out);
//         ROS_INFO("file 1 is open");
//         int cnt1 = 0;
//         for (int i = 0; i<leftTagDataArray.size(); i++)
//         {
//             data << std::fixed<< ++cnt1 << " " << leftTagDataArray[i].epcBuffer << " "<< leftTagDataArray[i].AntennaID << " " << leftTagDataArray[i].phase << " " << leftTagDataArray[i].timestamp << endl;
//         }
//         data.close();
//     }

//     char fileName2[200] = {0};
//     std::ofstream data1;
//     sprintf(fileName2, "//home//tzq//data//test//nav//RFIDdata_right.txt");
//     if (data1.fail())
//     {
//         ROS_INFO("Open file 2 failed");
//     }
//     else
//     {
//         data1.open(fileName2, std::ios::out);
//         ROS_INFO("file 2 is open");
//         int cnt2 = 0;
//         for (int i = 0; i<rightTagDataArray.size(); i++)
//         {
//             data1 << std::fixed<< ++cnt2 << " " << rightTagDataArray[i].epcBuffer << " "<< rightTagDataArray[i].AntennaID << " " << rightTagDataArray[i].phase << " " << rightTagDataArray[i].timestamp << endl;
//         }
//         data1.close();
//     }

//     char fileName3[200] = {0};
//     std::ofstream Odom;
//     sprintf(fileName3, "//home//tzq//data//test//nav//odom.txt");
//     if (Odom.fail())
//     {
//         ROS_INFO("Open file 3 failed");
//     }
//     else
//     {
//         Odom.open(fileName3, std::ios::out);
//         ROS_INFO("file 3 is open");
//         //int cnt1 = 0;
//         for (auto it1 = 0; it1 != odom_save.robot_x.size(); it1++)
//         {
//             Odom << std::fixed << odom_save.robot_x[it1] << " " << odom_save.robot_y[it1] << " " << odom_save.robot_th[it1] << " " << odom_save.robot_timestamp[it1] << endl;
//             //cnt1++;
//             // data1 << ++cnt1 << " " << (*it1).epc << " " << (*it1).channelIndex << " " << ((*it1).channelIndex - 1) * 0.25 + 920.625 << " " << (*it1).phase << " " << (*it1).rssi << endl;
//         }
//         Odom.close();
//     }
//     odom.robot_timestamp.swap(robotTimeStamp_recv);
//     odom.robot_x.swap(robotX_recv);
//     odom.robot_y.swap(robotY_recv);
//     odom.robot_th.swap(robotW_recv);

//     robotX_recv.clear();
//     robotY_recv.clear();
//     robotW_recv.clear();
//     robotTimeStamp_recv.clear();
    
//     double linear_x, angular;
//     if(control.iteration_count < 500)
//     {
//         tuple<double, double> motion = control.getMotion(leftTagDataArray, rightTagDataArray, odom, control.iteration_count);
//         linear_x = get<0>(motion);
//         angular = get<1>(motion);
//         // Tempolimits einhalten
//         if (linear_x > LINEAR_MAX_VEL)
//         {
//             linear_x = LINEAR_MAX_VEL;
//         }
//         else if (linear_x < -LINEAR_MAX_VEL)
//         {
//             linear_x = -LINEAR_MAX_VEL;
//         }

//         if (angular > ANGULAR_MAX_VEL)
//         {
//             angular = ANGULAR_MAX_VEL;
//         }
//         else if (angular < -ANGULAR_MAX_VEL)
//         {
//             angular = -ANGULAR_MAX_VEL;
//         }

//         control.vel_msg.linear.x = linear_x;
//         control.vel_msg.angular.z = angular;
//         ROS_INFO("[%0.2f m/s, %0.2f rad/s]", control.vel_msg.linear.x, control.vel_msg.angular.z);
//         // leftTagDataArray.clear();
//         // rightTagDataArray.clear();
//         control.iteration_count++;
//     }
// }

void multiThreadListener::vel_publish()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        motion_publish.publish(control.vel_msg);
        
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coreControl");
    multiThreadListener listener_obj;
    listener_obj.startRunning();

    ROS_INFO("Quit core Control.");
    ros::spin();
    return 0;
}
