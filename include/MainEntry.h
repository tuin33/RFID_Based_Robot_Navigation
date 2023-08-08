#pragma once
#ifndef MAINENTRY_H
#define MAINENTRY_H

#include "RFIDHandler_nav.h"
#include "StaticProperties.h"
// #include"StaticMethod.h"
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "time.h"
#include <string>
#include <fstream>
#include <vector>
#include <random>
#include "TagData.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define EIGEN_USE_MKL_ALL
#define EIGEN_VECTORIZE_SSE4_2

using namespace std;
using namespace Eigen;

typedef struct
{
    vector<double> robot_x;   // m
    vector<double> robot_y;   // m
    vector<double> robot_th;  // rad
    vector<double> robot_vel; // m/s
    vector<double> robot_timestamp;
} OdomData;
tuple<double, double, double, int> linear(double timestamp, OdomData odom);
void assignPoseForTagData(TagData *tagData, OdomData odom);

// PID controller
class Controller
{
public:
    // ofstream					outfile;
    CMyApplication myReaderApp;

    /* 机器人在每个迭代周期中的数据 */
    double robot_xt[10000];  // 单位为cm
    double robot_yt[10000];  // 单位为cm
    double robot_tht[10000]; // 单位为radian
    double robot_vel[10000]; // 单位为cm/s
    //double robot_distance[2000];

    /* 天线距离 */
    double distance_left_antenna[10000];
    double distance_right_antenna[10000];

    /* 标签信息 */
    double tag[2] = {300, 200};

    /* 机器人运动距离之和 */
    //double robot_distance_sum = 0.0;

    /* 定位的标签位置 */
    // double						tag_estimation_real_x[1000];
    // double						tag_estimation_real_y[1000];

    /* 所需标签信息解算起始点 */
    int num_length_start = 0;

    /* 迭代过程对应的TagDataArray的下标,当迭代第i次时，读到的标签总数为index_mapping[i] */
    int left_index_mapping[10000];
    int right_index_mapping[10000];

    /* 左右天线预处理后的相位值 */
    // double						phase_pos_left_antenna[1000];
    // double						phase_pos_right_antenna[1000];

    /* 传入Matlab engine的数据，每一行代表一组数据，每行中的第一的数据为处理后的相位，第二、三个数据为机器人的坐标x、y */
    double left_tag_phase_pos[10000][3];
    double right_tag_phase_pos[10000][3];

    /* 步长*/
    // double						step[2000];

    /* 迭代周期的启、始时刻*/
    // clock_t						startTime;
    // clock_t						endTime;
    //  LARGE_INTEGER	startTime;
    //  LARGE_INTEGER	endTime;

    /* 解算出的速度和角速度*/
    // double						robot_v[1000];
    // double						robot_w[1000];

    double phase_half_fuzzy_index_lb = 180 * 1 / 2;
    double phase_half_fuzzy_index_ub = 180 * 3 / 2;

    int phase_flag_left = 0;
    int phase_flag_right = 0;
    int phase_num_count_right = 0; // 记录当前相位的读取个数，一个相位观测值对应增加1
    int phase_num_count_left = 0;  // 记录当前相位的读取个数，一个相位观测值对应增加1
    int no_tag_cnt = 0;
    int no_tag_cnt_left = 0;
    int no_tag_cnt_right = 0;
    
    VectorXd phase_pre_left_antenna=VectorXd(10000);
    VectorXd phase_pre_right_antenna=VectorXd(10000);
    VectorXd phase_pre_left_antenna_2=VectorXd(10000);
    VectorXd phase_pre_right_antenna_2=VectorXd(10000);
    VectorXd phase_pos_left_antenna=VectorXd(10000);
    VectorXd phase_pos_right_antenna=VectorXd(10000);

    /* 天线的绝对坐标 */
    VectorXd antenna_left_real_xt=VectorXd(10000);
    VectorXd antenna_left_real_yt=VectorXd(10000);
    VectorXd antenna_right_real_xt=VectorXd(10000);
    VectorXd antenna_right_real_yt=VectorXd(10000);

    VectorXd tag_estimation_real_x=VectorXd(10000);
    VectorXd tag_estimation_real_y=VectorXd(10000);
    VectorXd tag_estimation_real_beta1=VectorXd(10000);
    VectorXd tag_estimation_real_beta2=VectorXd(10000);
    VectorXd resnorm=VectorXd(10000);

    VectorXd tag_estimation_relative_x=VectorXd(10000);
    VectorXd tag_estimation_relative_y=VectorXd(10000);

    VectorXd step=VectorXd(10000);
    VectorXd x_next=VectorXd(10000);
    VectorXd y_next=VectorXd(10000);
    VectorXd bearing_next=VectorXd(10000);
    VectorXd robot_w=VectorXd(10000);
    VectorXd robot_vx=VectorXd(10000);
    VectorXd robot_vy=VectorXd(10000);

    VectorXd tag_distance=VectorXd(10000); // 标签在机器人坐标系下的极坐标
    VectorXd tag_beta=VectorXd(10000);

    /*粒子滤波用的数据变量*/
    /*%粒子状态，第一、二行为标签x y值，第三行是左天线前一个时刻点的相位（解缠相位），第四行是左天线当前时刻点的相位（解缠相位），第5为左天线两个时刻之间的相位差值
    %第6行是右天线前一个时刻点的相位（解缠相位），第7行是右天线当前时刻点的相位（解缠相位），第8为右天线两个时刻之间的相位差值
    */
    MatrixXd PF_particle=MatrixXd(8, PF_count);

    MatrixXd PF_distance=MatrixXd(2, PF_count); // 粒子与观测值之间的差距,两个天线，故有两行
    MatrixXd PF_w=MatrixXd(3, PF_count);        // 粒子与观测值之间的权重因子，第三行用于综合评价

    MatrixXd PF_center_mean=MatrixXd(2, 10000); // 保存粒子的中心位置，第一行：x，第二行：y，第三行：前5次定位结果的x轴标准差，第四行：前5次定位结果的x轴标准差
    vector<MatrixXd> PF_point_all_x;
    vector<MatrixXd> PF_point_all_y;
    vector<MatrixXd> PF_point_all_x_after;
    vector<MatrixXd> PF_point_all_y_after;
    // VectorXd totaltime(2000);
    double totaltime[10000];
    /////!!!!!!!
    // string file_dic = "F:\\WuHaibing\\exp_data\\20180123\\2";

    int location_state = -1; // 初始化的定位状态。-1：数据累计阶段，不进行定位；0：定位阶段，但是定位结果不稳定；1：定位阶段，且结果稳定
    // PID控制
    double l_kp_, l_ki_, l_kd_ = 0;
    double a_kp_, a_ki_, a_kd_ = 0;
    double current_error_x_, current_error_sum_x_, current_error_diff_x_ = 0;
    double current_error_y_, current_error_sum_y_, current_error_diff_y_ = 0;
    double current_error_angle_, current_error_sum_angle_, current_error_diff_angle_ = 0;
    double last_error_x_, last_error_sum_x_, last_error_diff_x_ = 0;
    double last_error_y_, last_error_sum_y_, last_error_diff_y_ = 0;
    double last_error_angle_, last_error_sum_angle_, last_error_diff_angle_ = 0;

    double left_g,right_g;
    int flag_g=0;
    vector<double> delta_g;
    vector<double> left_g_save;
    vector<double> right_g_save;

    Controller();  
    int iteration_count=0;
    geometry_msgs::Twist vel_msg;


    // PID控制
    double pid_x_compute(double real_position, double target_position);
    double pid_y_compute(double real_position, double target_position);
    double pid_angle_compute(double left_g, double right_g);

    void getGradient(int i,vector<TagData> leftTagDataArray, vector<TagData> rightTagDataArray);

    // 粒子滤波
    tuple<double, double> getMotion(vector<TagData>* leftTagDataArray, vector<TagData>* rightTagDataArray, OdomData odom, int i);
};
#endif
