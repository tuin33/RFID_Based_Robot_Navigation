/**************************************************************************
** 标签信息结构体
**
**************************************************************************/
#ifndef _TAG_DATA_H_
#define _TAG_DATA_H_

#include<string>
using std::string;
typedef struct TagDataStruct
{
	int					index;						//编号，便于数据的显示和处理
	bool				isPreTreated = false;		//判断标签的相位信息是否已被预处理
	bool				isAssignedPose = false;		//判断标签机器人坐标信息是否已赋值
	string				epcBuffer;					//标签EPC
	double				phase;						//相位
	double				frequency;					//频率
	double				timestamp;					//firstSeenTimestampUTC
	unsigned long long	timestamp_pc;				//记录电脑取出标签信息的时间戳
	double				x=0.0;						//读取到标签信息的时刻，天线的绝对坐标-X坐标
	double				y=0.0;						//读取到标签信息的时刻，天线的绝对坐标-Y坐标
	unsigned short		 AntennaID;					//读写器ID，左侧读写器ID为1，右侧读写器ID为2
	double				robotX =0.0;						//机器人坐标-X m
	double				robotY=0.0;						//机器人坐标-Y m
	double				robotTh=0.0;					//机器人朝向角（弧度）-Th
}TagData;


#endif
