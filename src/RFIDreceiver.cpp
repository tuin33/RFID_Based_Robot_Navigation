#include "ros/ros.h"
#include "RFID_Based_Robot_Navigation/RFIDdata.h"
#include "TagData.h"

//回调函数部分
void chatterCallback(const RFID_Based_Robot_Navigation::RFIDdata::ConstPtr& msg)
{
    ROS_INFO("Receiving from tag: epc: %s, ant: %d", msg->epc.c_str(), msg->antID); //将接收到的消息打印出来
    TagData tagData;
    std::vector<TagData> leftTagDataArray;
    std::vector<TagData> rightTagDataArray; 
    for (int i = 0; i < msg->phase.size(); i++)
    {
        tagData.epcBuffer = msg->epc;
        tagData.AntennaID = msg->antID;
        tagData.phase = msg->phase[i];
        tagData.timestamp = msg->timestamp[i];
        if (msg->antID == 1)
        {
            leftTagDataArray.push_back(tagData);
        }
        else if (msg->antID ==2)
        {
            rightTagDataArray.push_back(tagData);
        }
    }
    std::cout<<"Antenna"<<msg->antID <<" :TagDataArray is recieved!"<<std::endl;
    for(int i = 0; i <leftTagDataArray.size(); i++)
    {
        std::cout<<leftTagDataArray[i].epcBuffer<<"   "<<leftTagDataArray[i].phase<<"   "<<leftTagDataArray[i].timestamp<<std::endl;
    }
    for(int i = 0; i <rightTagDataArray.size(); i++)
    {
        std::cout<<rightTagDataArray[i].epcBuffer<<"   "<<rightTagDataArray[i].phase<<"   "<<rightTagDataArray[i].timestamp<<std::endl;
    }
}
/*subscriber的回调函数，当接收到 chatter 话题的时候就会被调用。
参数是所接收的消息的常数指标（const pkg_name::msg_name::ConstPtr& msg）.
消息是以 boost shared_ptr 指针的形式传输，这就意味着你可以存储它而又不需要复制数据。
之后使用msg->field_name即可存取message的资料
*/

int main(int argc, char **argv){
    ros::init(argc, argv, "listener"); //初始化ROS节点
    ros::NodeHandle nh; //创建句柄节点
    ros::Subscriber sub = nh.subscribe("/Data_RFID", 50, chatterCallback);
/*告诉 master 要订阅 chatter 话题（第一个参数）上的消息。
当有消息发布到这个话题时，ROS 就会调用 chatterCallback() 函数（第三个参数）。
第二个参数是队列大小，当缓存达到 1000 条消息后，自动舍弃时间戳最早的消息。
NodeHandle::subscribe() 返回 ros::Subscriber 对象,此处为sub。
当这个对象销毁时，它将自动退订 chatter 话题的消息。
有各种不同的 NodeHandle::subscribe() 函数，可以指定类的成员函数，甚至是 Boost.Function 对象可以调用的任何数据类型。
*/
    ros::spin();
/*ros::spin() 进入自循环，可以尽可能快的调用消息回调函数，会调用主程序中所有回调函数，此处只有chatterCallback()
一旦 ros::ok() 返回 false，ros::spin() 就会立刻跳出自循环。
这有可能是 ros::shutdown() 被调用，或者是用户按下了 Ctrl-C，使得 master 告诉节点要终止运行。
*/
    return 0;
}
