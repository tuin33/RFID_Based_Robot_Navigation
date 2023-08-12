#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"    //包含了使用的数据类型
#include "stdlib.h"
#include "stdio.h"
#include "unistd.h"
#include "signal.h"
#include "sys/types.h"
#include "sys/wait.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
 
class VoicecmdSub{
	private:
		ros::NodeHandle n;
		std_msgs::Int32  cmd;
		ros::Subscriber voicesub;	//定义订阅着
		ros::Publisher 	cmdpub;		//定义发布者
		std_msgs::Int32 oldmsg;
        geometry_msgs::Twist vel_msg;
		bool opennav;                //开始导航标志

		pid_t pid_nav;				//导航进程号

	public:
		VoicecmdSub()
		{
			cmdpub = n.advertise<geometry_msgs::Twist>("/cmd_vel",20);//发布到“keycmd”上
			voicesub = n.subscribe("/audio_info",100,&VoicecmdSub::voicecmdCallback,this);//订阅“voicewords”主题
			oldmsg .data= -1;
			opennav = false;

			pid_nav = -1;

		}
		~VoicecmdSub(){}
		void voicecmdCallback(const std_msgs::Int32::ConstPtr& msgs)
		{
			std::stringstream ss;
			ss<<"";
			if(msgs->data!= oldmsg.data )
			{	
				oldmsg.data = msgs->data;
				std::cout<<"I Heard:"<<msgs->data<<std::endl;
				
				//通过识别的命令执行对应的操作，利用system函数创建进程播放合成的语音
	
				if(msgs->data == 1)
				{
					//system("mplayer /home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/music/cmdrcv.wav");
					ss<<"go";
                    vel_msg.linear.x=0.1;
                    vel_msg.angular.z=0;
                    cmdpub.publish(vel_msg);
				}
				else if(msgs->data== 2)
				{
					//system("mplayer /home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/music/cmdrcv.wav");
					ss<<"back";
                    vel_msg.linear.x=-0.1;
                    vel_msg.angular.z=0;
                    cmdpub.publish(vel_msg);
				}
				else if(msgs->data == 3)
				{
					//system("mplayer /home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/music/cmdrcv.wav");
					ss<<"left";
                    vel_msg.linear.x=0;
                    vel_msg.angular.z=0.5;
                    cmdpub.publish(vel_msg);
				}
				else if(msgs->data ==4)
				{
					//system("mplayer /home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/music/cmdrcv.wav");
					ss<<"right";
                    vel_msg.linear.x=0;
                    vel_msg.angular.z=-0.5;
                    cmdpub.publish(vel_msg);
				}
                else if(msgs->data == 0)
				{
					//system("mplayer /home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/music/cmdrcv.wav");
					ss<<"right";
                    vel_msg.linear.x=0;
                    vel_msg.angular.z=0;
                    cmdpub.publish(vel_msg);
				}
				else if(msgs->data ==5)
				{
					//system("mplayer /home/tzq/catkin_ws/src/RFID_Based_Robot_Navigation/music/opennav.wav");
					ss<<"opennav";
					if(!opennav)
					{
						opennav = true;
						pid_nav = fork();//创建子进程执行操作
						if(pid_nav == 0)
						{	
							// 执行source ~catkin_ws/devel/setup.bash
							execl("/bin/bash","bash","-c","source /home/haoran/catkin_ws/devel/setup.bash",NULL);
                            execl("/opt/ros/noetic/bin/rosrun","rosrun","RFID_Based_Robot_Navigation","coreConrtol_cp",NULL);
						}
					}	
				}				
            
				//cmd.data = ss.str();
				//cmdpub.publish(cmd);//发布消息
			}
			
		}
};		
 
int main(int argc,char** argv)
{
	ros::init(argc,argv,"voice_pub");
	VoicecmdSub it;
	ros::spin();
	return 0;
}