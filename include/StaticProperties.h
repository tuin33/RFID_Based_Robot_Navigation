/**
*******************************************************************
** @brief 集合了工程中使用到的全局性常量，可以方便管理和配置参数
**
** @author hongc 2017.07.05
*******************************************************************/
#ifndef _STATIC_PROPERTIES_H_
#define _STATIC_PROPERTIES_H_

#define	RFID_READER_IP_ADDRESS	"115.156.142.223"					//192.168.0.105RFID读写器的IP地址

#define	LEFT_READER_ID			1									//左读写器ID
#define	RIGHT_READER_ID			2									//右读写器ID

#define	TARGET_TAG_EPC			"E280-1160-6000-0210-BE88-B02F"		//目标标签EPC	3008-33B2-DDD9-0140-2018-0118
//#define TARGET_TAG_EPC          "E200-001A-0411-0175-1050-96C3"
#define ARRAY_SIZE				1000000								//保存数组的大小

#define PI						3.1415926
#define	antenna_H_left				25.0/100//27.08//24.1
#define	antenna_alpha_left			0 //(1.5707963+0.0739)//67.2/180*PI
#define	antenna_H_right				25.0/100//29.07//25.2
#define	antenna_alpha_right			0 //(1.5707963+0.0689)//67.0/180*PI

#define antenna_left_x				0.0/100
#define antenna_left_y			    -25.0/100
#define antenna_right_x				1.0/100
#define antenna_right_y			    0.0/100

#define	control_period			0.1									//控制周期，单位秒 
#define	sample_total_len		20.0/100									//初始的采样长度，m
#define phase_grade_len			6.0/100									//相位梯度对应的物理距离，m

#define	tag_target_relative_x	5.0/100								//m
#define tag_target_relative_y	0.0									//m

#define tag_distance_threshold  40.0/100				//m
#define tag_beta_threshold	10.0 / 180.0 * PI		//rad

#define	robot_translational_vel0	4.0/100								//m/s
#define robot_rotational_vel0		0.0								

#define true 1
#define false 0

#define PF_count		1000		//粒子滤波的粒子数量
#define PF_scope_x_min	0			//定位区域x坐标最小值
#define PF_scope_x_max	5			//定位区域x坐标最大值
#define PF_scope_y_min	-1		//定位区域y坐标最小值
#define PF_scope_y_max	1			//定位区域y坐标最大值
#define PF_Q			0.03			//过程噪声，标准差，单位m
#define PF_R			pow(0.1,2)	//测量噪声，方差，单位rad

#define LINEAR_MAX_VEL 0.5  //速度最大值
#define ANGULAR_MAX_VEL 0.3

#endif