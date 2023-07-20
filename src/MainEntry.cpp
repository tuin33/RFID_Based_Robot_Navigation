#include "MainEntry.h"

Controller::Controller()
{
	l_kp_ = 0.08;

	a_kp_ = 0.08;
}

/**
***************************************************************************************************
**
** @brief 通过标签信息中的时间戳获取到机器人在该时间戳下的位置，并赋值给标签信息。若标签坐标信息已赋值则跳过
**
***************************************************************************************************
**/
tuple<double, double, double, int> linear(double timestamp, OdomData odom)
{
	double robot_x = 0, robot_y = 0, robot_th = 0;
	int resultCode = 0;
	for (int i = 1; i < odom.robot_timestamp.size(); i++)
	{
		if (timestamp < odom.robot_timestamp[i])
		{
			robot_x = (odom.robot_x[i] - odom.robot_x[i - 1]) / (odom.robot_timestamp[i] - odom.robot_timestamp[i - 1]) * (timestamp - odom.robot_timestamp[i - 1]) + odom.robot_x[i - 1];
			robot_y = (odom.robot_y[i] - odom.robot_y[i - 1]) / (odom.robot_timestamp[i] - odom.robot_timestamp[i - 1]) * (timestamp - odom.robot_timestamp[i - 1]) + odom.robot_y[i - 1];
			robot_th = (odom.robot_th[i] - odom.robot_th[i - 1]) / (odom.robot_timestamp[i] - odom.robot_timestamp[i - 1]) * (timestamp - odom.robot_timestamp[i - 1]) + odom.robot_th[i - 1];
			resultCode = 1;
			break;
		}
	}
	return make_tuple(robot_x, robot_y, robot_th, resultCode);
}

void assignPoseForTagData(TagData *tagData, OdomData odom)
{
	if (tagData->isAssignedPose)
		return;
	tuple<double, double, double, int> getPose = linear(tagData->timestamp, odom);
	int resultCode = get<3>(getPose);
	if (resultCode != 1)
	{
		cout << "Warn：AssignmentPoseForTagData Exception!"
			 << " return code: " << resultCode << endl;
		//  << "(天线编号，数组下标) is"
		//  << "(" << tagData->readerID << "," << tagData->index << ")"
		//  << "ArTime:" << tagData->timestamp_robot.getSec() << '.' << tagData->timestamp_robot.getMSec() << endl;
	}
	else
	{
		double robot_x = get<0>(getPose);
		double robot_y = get<1>(getPose);
		double robot_th = get<2>(getPose);
		if (tagData->AntennaID == LEFT_READER_ID)
		{
			tagData->x = robot_x - antenna_H_left * cos(PI - antenna_alpha_left - robot_th);
			tagData->y = robot_y + antenna_H_left * sin(PI - antenna_alpha_left - robot_th);
			tagData->robotX = robot_x;
			tagData->robotY = robot_y;
			tagData->robotTh = robot_th;
			// cout << "Info: left TagData pose assigned! x=" <<tagData->x<<" y="<<tagData->y<<"th= "<<tagData->robotTh<<endl;
		}
		else if (tagData->AntennaID == RIGHT_READER_ID)
		{
			tagData->x = robot_x + antenna_H_right * cos(-antenna_alpha_right + robot_th);
			tagData->y = robot_y + antenna_H_right * sin(-antenna_alpha_right + robot_th);
			tagData->robotX = robot_x;
			tagData->robotY = robot_y;
			tagData->robotTh = robot_th;
			// cout << "Info: right TagData pose assigned! x=" <<tagData->x<<" y="<<tagData->y<< "th= "<<tagData->robotTh<<endl;
		}
		tagData->isAssignedPose = true;
	}
	return;
}

double Controller::pid_x_compute(double real_position, double target_position)
{
	current_error_x_ = target_position - real_position;
	current_error_sum_x_ += current_error_x_;
	current_error_diff_x_ = current_error_x_ - last_error_x_;

	double output = l_kp_ * current_error_x_ + l_ki_ * current_error_sum_x_ + l_kd_ * current_error_diff_x_;

	last_error_x_ = current_error_x_;
	last_error_sum_x_ = current_error_sum_x_;
	last_error_diff_x_ = current_error_diff_x_;

	return output;
}

double Controller::pid_y_compute(double real_position, double target_position)
{
	current_error_y_ = target_position - real_position;
	current_error_sum_y_ += current_error_y_;
	current_error_diff_y_ = current_error_y_ - last_error_y_;

	double output = l_kp_ * current_error_y_ + l_ki_ * current_error_sum_y_ + l_kd_ * current_error_diff_y_;

	last_error_y_ = current_error_y_;
	last_error_sum_y_ = current_error_sum_y_;
	last_error_diff_y_ = current_error_diff_y_;

	return output;
}

double Controller::pid_angle_compute(double real_angle, double target_angle)
{
	current_error_angle_ = target_angle - real_angle;
	current_error_sum_angle_ += current_error_angle_;
	current_error_diff_angle_ = current_error_angle_ - last_error_angle_;

	double output = a_kp_ * current_error_angle_ + a_ki_ * current_error_sum_angle_ + a_kd_ * current_error_diff_angle_;

	last_error_angle_ = current_error_angle_;
	last_error_sum_angle_ = current_error_sum_angle_;
	last_error_diff_angle_ = current_error_diff_angle_;

	return output;
}

tuple<double, double> Controller::getMotion(vector<TagData> *leftTagDataArray, vector<TagData> *rightTagDataArray, OdomData odom, int i)
{
	// vector<double> dataVector;
	/* 记录某一时刻的机器人时间和系统时间戳
	const ArTime currntArTime;
	ftime(&currentSysTime);
	**********************************/
	cout << "Info: loop " << i << " started" << endl;

	srand((unsigned)time(NULL)); // 设定种子数--为了在c语言中生成随机数，配合rand使用

	/* 获取机器人的位置和姿态并进行单位换算 cm rad cm/s*/
	int odomNum = odom.robot_x.size();

	robot_xt[i] = odom.robot_x[odomNum - 1];
	robot_yt[i] = odom.robot_y[odomNum - 1]; // m-->cm
	robot_tht[i] = odom.robot_th[odomNum - 1];
	cout << "robot_xt[i]: " << robot_xt[i] << endl;
	cout << "robot_yt[i]: " << robot_yt[i] << endl;
	cout << "robot_tht[i]: " << robot_tht[i] << endl;
	// robot_vel[i] = odom.robot_vel[odomNum-1] * 100; // m/s-->cm/s

	std::cout << "Info: robot pose and velocity got" << std::endl;
	/* 获取当前读到的最新标签在数组中的下标(个数)，index */
	left_index_mapping[i] = leftTagDataArray->size() - 1;
	right_index_mapping[i] = rightTagDataArray->size() - 1;
	for (int p = (i > 3 ? left_index_mapping[i - 3] : 0); p <= left_index_mapping[i]; p++)
	{
		assignPoseForTagData(&(leftTagDataArray->at(p)), odom);
	}
	for (int p = (i > 3 ? right_index_mapping[i - 3] : 0); p <= right_index_mapping[i]; p++)
	{
		assignPoseForTagData(&(rightTagDataArray->at(p)), odom);
	}
	std::cout << "Info: assignPoseForTagData finished" << std::endl;
	/*由于硬件测量原因，需要先调整相位的值,进而对相位进行预处理*/

	if ((phase_flag_left == 0) && (left_index_mapping[i] >= 0))
	{
		// 左天线
		phase_flag_left = 1;
		phase_pre_left_antenna(phase_num_count_left) = leftTagDataArray->at(0).phase;
		phase_pre_left_antenna_2(phase_num_count_left) = (360 - phase_pre_left_antenna(phase_num_count_left)) / 180.0 * PI;
		phase_pos_left_antenna(phase_num_count_left) = phase_pre_left_antenna_2(phase_num_count_left);

		for (int j = 1; j <= left_index_mapping[i]; j++)
		{
			double diff = leftTagDataArray->at(j).phase - phase_pre_left_antenna(phase_num_count_left);
			if (diff > phase_half_fuzzy_index_lb && diff < phase_half_fuzzy_index_ub)
			{
				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray->at(j).phase - 180;
			}
			else if ((-diff) > phase_half_fuzzy_index_lb && (-diff) < phase_half_fuzzy_index_ub)
			{
				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray->at(j).phase + 180;
			}
			else
			{
				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray->at(j).phase;
			}
			phase_pre_left_antenna_2(phase_num_count_left) = (360 - phase_pre_left_antenna(phase_num_count_left)) / 180.0 * PI;
			int k = round((phase_pre_left_antenna_2(phase_num_count_left) - phase_pos_left_antenna(phase_num_count_left - 1)) / (2 * PI));
			phase_pos_left_antenna(phase_num_count_left) = phase_pre_left_antenna_2(phase_num_count_left) - 2 * PI * k;
		}
		cout << "左天线相位初始化完成 " << endl;
	}
	else if (phase_flag_left == 1)
	{
		// 左天线
		for (int j = left_index_mapping[i - 1] + 1; j <= left_index_mapping[i]; j++)
		{
			double diff = leftTagDataArray->at(j).phase - phase_pre_left_antenna(phase_num_count_left);
			if (diff > phase_half_fuzzy_index_lb && diff < phase_half_fuzzy_index_ub)
			{
				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray->at(j).phase - 180;
			}
			else if ((-diff) > phase_half_fuzzy_index_lb && (-diff) < phase_half_fuzzy_index_ub)
			{
				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray->at(j).phase + 180;
			}
			else
			{
				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray->at(j).phase;
			}
			phase_pre_left_antenna_2(phase_num_count_left) = (360 - phase_pre_left_antenna(phase_num_count_left)) / 180.0 * PI;
			;
			int k = round((phase_pre_left_antenna_2(phase_num_count_left) - phase_pos_left_antenna(phase_num_count_left - 1)) / (2 * PI));
			phase_pos_left_antenna(phase_num_count_left) = phase_pre_left_antenna_2(phase_num_count_left) - 2 * PI * k;
		}
	}

	if ((phase_flag_right == 0) && (right_index_mapping[i] >= 0))
	{
		// 右天线
		phase_flag_right = 1;
		phase_pre_right_antenna(phase_num_count_right) = rightTagDataArray->at(0).phase;
		phase_pre_right_antenna_2(phase_num_count_right) = (360 - phase_pre_right_antenna(phase_num_count_right)) / 180.0 * PI;
		phase_pos_right_antenna(phase_num_count_right) = phase_pre_right_antenna_2(phase_num_count_right);

		for (int j = 1; j <= right_index_mapping[i]; j++)
		{
			double diff = rightTagDataArray->at(j).phase - phase_pre_right_antenna(phase_num_count_right);
			if (diff > phase_half_fuzzy_index_lb && diff < phase_half_fuzzy_index_ub)
			{
				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray->at(j).phase - 180;
			}
			else if ((-diff) > phase_half_fuzzy_index_lb && (-diff) < phase_half_fuzzy_index_ub)
			{
				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray->at(j).phase + 180;
			}
			else
			{
				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray->at(j).phase;
			}
			phase_pre_right_antenna_2(phase_num_count_right) = (360 - phase_pre_right_antenna(phase_num_count_right)) / 180.0 * PI;
			int k = round((phase_pre_right_antenna_2(phase_num_count_right) - phase_pos_right_antenna(phase_num_count_right - 1)) / (2 * PI));
			phase_pos_right_antenna(phase_num_count_right) = phase_pre_right_antenna_2(phase_num_count_right) - 2 * PI * k;
		}
		cout << "右天线相位初始化完成 " << endl;
	}
	else if (phase_flag_right == 1)
	{

		// 右天线
		for (int j = right_index_mapping[i - 1] + 1; j <= right_index_mapping[i]; j++)
		{
			double diff = rightTagDataArray->at(j).phase - phase_pre_right_antenna(phase_num_count_right);
			if (diff > phase_half_fuzzy_index_lb && diff < phase_half_fuzzy_index_ub)
			{
				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray->at(j).phase - 180;
			}
			else if ((-diff) > phase_half_fuzzy_index_lb && (-diff) < phase_half_fuzzy_index_ub)
			{
				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray->at(j).phase + 180;
			}
			else
			{
				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray->at(j).phase;
			}
			phase_pre_right_antenna_2(phase_num_count_right) = (360 - phase_pre_right_antenna(phase_num_count_right)) / 180.0 * PI;
			int k = round((phase_pre_right_antenna_2(phase_num_count_right) - phase_pos_right_antenna(phase_num_count_right - 1)) / (2 * PI));
			phase_pos_right_antenna(phase_num_count_right) = phase_pre_right_antenna_2(phase_num_count_right) - 2 * PI * k;
		}
	}
	cout << "左天线相位当前数量: " << phase_num_count_left << endl;
	cout << "右天线相位当前数量: " << phase_num_count_right << endl;
	for(int i=0;i<phase_num_count_left;i++)
	{
		cout<<"左天线相位: "<<phase_pos_left_antenna(i)<<endl;
	}
	for(int i=0;i<phase_num_count_right;i++)
	{
		cout<<"右天线相位: "<<phase_pos_right_antenna(i)<<endl;
	}
	// cout<<"poseX:"<<robotPose.pose.pose.position.x<<endl;
	cout << "location_state:" << location_state << endl;
	if ((robot_xt[i] < sample_total_len) && (location_state == -1)) // 机器人的运动距离未达到定位的标准，且处于数据累计阶段
	{
		cout << "Particle Initial." << endl;
		PF_particle.row(0) = PF_scope_x_min + (PF_scope_x_max - PF_scope_x_min) / 2 * (MatrixXd::Random(1, PF_count).array() + 1);
		PF_particle.row(1) = PF_scope_y_min + (PF_scope_y_max - PF_scope_y_min) / 2 * (MatrixXd::Random(1, PF_count).array() + 1);
		// std::cout << "x:" << std::endl;
		// std::cout << PF_particle.row(0) << std::endl;
		// std::cout << "Y:" << std::endl;
		// std::cout << PF_particle.row(1) << std::endl;
		PF_center_mean(0, i) = 0; // 计算粒子的x轴坐标均值
		PF_center_mean(1, i) = 0;

		// PF_center_mean(0, i) = PF_particle.block(0, 0, 1, PF_count).sum() / PF_count; // 计算粒子的x轴坐标均值
		// PF_center_mean(1, i) = PF_particle.block(1, 0, 1, PF_count).sum() / PF_count;
		// cout<<"第"<<i<<"次定位结果为:x="<<PF_center_mean(0, i)<<" y="<<PF_center_mean(1, i)<<endl;
		tag_estimation_real_x(i) = 0;
		tag_estimation_real_y(i) = 0;

		step(i) = -1;

		robot_vx(i) = robot_translational_vel0;
		robot_vy(i) = 0;
		robot_w(i) = robot_rotational_vel0;
	}
	else
	{
		cout << "Begin PF." << endl;
		random_device rd;
		mt19937 gen(rd());
		normal_distribution<double> normal(0, PF_Q); // 均值 和 标准差
		// normal(gen);

		for (int j = 0; j < PF_count; j++)
		{
			// double ran1 = normal(gen);
			PF_particle(0, j) = PF_particle(0, j) + normal(gen);
			// double ran2 = normal(gen);
			PF_particle(1, j) = PF_particle(1, j) + normal(gen);
		}
		// cout << "1" << endl;

		/*确定最后一个有效测量点的下标*/
		int leftEnd = left_index_mapping[i];
		int rightEnd = right_index_mapping[i];

		while (!leftTagDataArray->at(leftEnd).isAssignedPose)
		{
			leftEnd--;
		}
		while (!rightTagDataArray->at(rightEnd).isAssignedPose)
		{
			rightEnd--;
		}
		// cout << "2" << endl;
		/*确定距离 相位梯度 距离点的测量点的下标*/
		int leftStart = leftEnd;
		int rightStart = rightEnd;
		while (1)
		{
			double left_distance_interval = sqrt(pow(leftTagDataArray->at(leftEnd).x - leftTagDataArray->at(leftStart).x, 2) + pow(leftTagDataArray->at(leftEnd).y - leftTagDataArray->at(leftStart).y, 2));
			double right_distance_interval = sqrt(pow(rightTagDataArray->at(rightEnd).x - rightTagDataArray->at(rightStart).x, 2) + pow(rightTagDataArray->at(rightEnd).y - rightTagDataArray->at(rightStart).y, 2));
			if ((left_distance_interval > 6.0 / 100) || (right_distance_interval > 6.0 / 100))
			{
				break;
			}
			else
			{
				leftStart = leftStart - 1;
				rightStart = rightStart - 1;
			}
			if ((leftStart < 2) || (rightStart < 2))
			{
				break;
			}
			double left_phase_dis = sqrt(pow(leftTagDataArray->at(leftStart).x - leftTagDataArray->at(leftStart + 1).x, 2) + pow(leftTagDataArray->at(leftStart).y - leftTagDataArray->at(leftStart + 1).y, 2));
			double right_phase_dis = sqrt(pow(rightTagDataArray->at(rightStart).x - rightTagDataArray->at(rightStart + 1).x, 2) + pow(rightTagDataArray->at(rightStart).y - rightTagDataArray->at(rightStart + 1).y, 2));
			if ((left_phase_dis > 3.0 / 100) || (right_phase_dis > 3.0 / 100))
			{
				leftStart = leftStart + 1;
				rightStart = rightStart + 1;
				break;
			}
		}
		// cout << "3" << endl;
		double wave_length_var[16] = {32.5866 / 100, 32.5777, 32.5689, 32.56, 32.5512, 32.5424, 32.5336, 32.5247, 32.5159, 32.5071, 32.4983, 32.4895, 32.4807, 32.4719, 32.4631, 32.4544}; // 波长，cm
		/*左右天线前一时刻相位*/
		RowVectorXd PF_distance_left_antenna_last(PF_count);	  // 行向量
		RowVectorXd PF_distance_right_antenna_last(PF_count);  // 行向量
		RowVectorXd PF_distance_left_antenna_now(PF_count);  // 行向量
		RowVectorXd PF_distance_right_antenna_now(PF_count); // 行向量
		PF_distance_left_antenna_last= ((PF_particle.row(0).array() - leftTagDataArray->at(leftStart).x).array().pow(2) + (PF_particle.row(1).array() - leftTagDataArray->at(leftStart).y).array().pow(2)).cwiseSqrt().array() * 2;
		PF_distance_left_antenna_last = ((rightTagDataArray->at(rightStart).x - PF_particle.row(0).array()).array().pow(2) + (rightTagDataArray->at(rightStart).y - PF_particle.row(1).array()).array().pow(2)).cwiseSqrt().array() * 2;
		PF_particle.row(2) = ((PF_distance_left_antenna_last.array() / wave_length_var[0]) - (PF_distance_left_antenna_last.array() / wave_length_var[0]).floor()).array() * 2 * PI;
		PF_particle.row(5) = ((PF_distance_right_antenna_last.array() / wave_length_var[0]) - (PF_distance_right_antenna_last.array() / wave_length_var[0]).floor()).array() * 2 * PI;
		cout << "-------------------------------------------------------------------------------------" << endl;
		cout << "预测上一时刻左天线解缠相位为 " << PF_distance_left_antenna_last.array().sum() / wave_length_var[0]/PF_count * 2 * PI << endl;
		cout << "PF_particle.row(2): " << PF_particle.row(2).sum() /PF_count << endl;
		cout << "预测上一时刻右天线解缠相位为 " << PF_distance_right_antenna_last.array().sum() / wave_length_var[0]/PF_count * 2 * PI << endl;
		cout << "PF_particle.row(5): " << PF_particle.row(5).sum() /PF_count << endl;

		///*左右天线当前时刻相位*/
		PF_distance_left_antenna_now = ((leftTagDataArray->at(leftEnd).x - PF_particle.row(0).array()).array().pow(2) + (leftTagDataArray->at(leftEnd).y - PF_particle.row(1).array()).array().pow(2)).cwiseSqrt().array() * 2;
		PF_distance_right_antenna_now = ((rightTagDataArray->at(rightEnd).x - PF_particle.row(0).array()).array().pow(2) + (rightTagDataArray->at(rightEnd).y - PF_particle.row(1).array()).array().pow(2)).cwiseSqrt().array() * 2;

		PF_particle.row(3) = ((PF_distance_left_antenna_now.array() / wave_length_var[0]) - (PF_distance_left_antenna_now.array() / wave_length_var[0]).floor()) * 2 * PI;
		PF_particle.row(6) = ((PF_distance_right_antenna_now.array() / wave_length_var[0]) - (PF_distance_right_antenna_now.array() / wave_length_var[0]).floor()) * 2 * PI;
		cout << "-------------------------------------------------------------------------------------" << endl;
		cout << "预测当前时刻左天线解缠相位为 " << PF_distance_left_antenna_now.array().sum() / wave_length_var[0] /PF_count* 2 * PI << endl;
		cout << "PF_particle.row(3): " << PF_particle.row(3).sum() /PF_count << endl;
		cout << "预测当前时刻右天线解缠相位为 " << PF_distance_right_antenna_now.array() .sum()/ wave_length_var[0]/PF_count * 2 * PI << endl;
		cout << "PF_particle.row(6): " << PF_particle.row(6).sum() /PF_count << endl;

		///*左右天线当前相位梯度*/
		RowVectorXd k(PF_count);
		k = ((PF_particle.row(3) - PF_particle.row(2)).array() / (2 * PI)).round();
		PF_particle.row(4) = PF_particle.row(3).array() - 2 * PI * k.array();
		PF_particle.row(4) = PF_particle.row(4).array() - PF_particle.row(2).array();
		k = ((PF_particle.row(6) - PF_particle.row(5)).array() / (2 * PI)).round();
		PF_particle.row(7) = PF_particle.row(6).array() - 2 * PI * k.array();
		PF_particle.row(7) = PF_particle.row(7).array() - PF_particle.row(5).array();
		cout << "预测左天线解缠相位为 " << PF_particle.row(4).sum()/PF_count  << endl;
		cout << "预测右天线解缠相位为" << PF_particle.row(7).sum()/PF_count  << endl;
		// cout << "3.3" << endl;
		/*实际观测相位梯度*/
		double PF_observe_left = phase_pos_left_antenna(leftEnd) - phase_pos_left_antenna(leftStart);
		double PF_observe_right = phase_pos_right_antenna(rightEnd) - phase_pos_right_antenna(rightStart);
		cout << "-------------------------------------------------------------------------------------" << endl;
		cout << "实际观测左天线解缠相位为 " << PF_observe_left << endl;
		cout <<"phase_pos_left_antenna(leftEnd)=" << phase_pos_left_antenna(leftEnd) << "	phase_pos_left_antenna(leftStart)=" << phase_pos_left_antenna(leftStart) << endl;
		cout << "实际观测右天线解缠相位为 " << PF_observe_right << endl;
		cout << "phase_pos_right_antenna(rightEnd)=" << phase_pos_right_antenna(rightEnd) << "	phase_pos_right_antenna(rightStart)=" << phase_pos_right_antenna(rightStart) << endl;
		/*粒子权重评估*/
		PF_distance.row(0) = PF_particle.row(4).array() - PF_observe_left;
		PF_w.row(0) = (1 / sqrt(2 * PF_R) / sqrt(2 * PI)) * (-PF_distance.row(0).array().pow(2).array() / (4 * PF_R)).array().exp(); // 求权重

		PF_distance.row(1) = PF_particle.row(7).array() - PF_observe_right;
		PF_w.row(1) = (1 / sqrt(2 * PF_R) / sqrt(2 * PI)) * (-PF_distance.row(1).array().pow(2).array() / (4 * PF_R)).array().exp(); // 求权重

		PF_w.row(2) = PF_w.row(0).array() * PF_w.row(1).array(); // 综合权重
		cout << "-------------------------------------------------------------------------------------" << endl;
		cout << "PF_distance_x_mean" << PF_distance.row(0).sum() /PF_count<< endl;
		cout << "PF_distance_y_mean: " << PF_distance.row(1).sum() /PF_count<< endl;
		// cout << "4" << endl;
		/*重采样*/
		MatrixXd PF_particle_new(8, PF_count);
		int index = (((double)rand()) / RAND_MAX) * (PF_count - 1) + 0; // 生成0 - （PF_count-1） 的随机数
		double beta = 0;
		double mw = PF_w.row(2).maxCoeff();
		for (int j = 0; j < PF_count; j++)
		{
			beta = beta + 2 * mw * (((double)rand()) / RAND_MAX);
			while (beta > PF_w(2, index))
			{
				beta = beta - PF_w(2, index);
				index = (index + 1) % PF_count;
			}
			PF_particle_new.col(j) = PF_particle.col(index);
		}
		PF_particle = PF_particle_new;
		// cout << "5" << endl;
		///*定位结果*/
		PF_center_mean(0, i) = PF_particle.block(0, 0, 1, PF_count).sum() / PF_count; // 计算粒子的x轴坐标均值
		PF_center_mean(1, i) = PF_particle.block(1, 0, 1, PF_count).sum() / PF_count; // 计算粒子的y轴坐标均值
		cout << "-------------------------------------------------------------------------------------" << endl;
		cout << "第" << i << "次定位结果为:x=" << PF_center_mean(0, i) << " y=" << PF_center_mean(1, i) << endl;
		cout << "-------------------------------------------------------------------------------------" << endl;
		int num = 5;

		if (i >= num - 1)
		{
			double mean_x, mean_y, std_x, std_y;
			if (location_state != 1)
			{
				mean_x = PF_center_mean.block(0, i - num + 1, 1, num).sum() / num;
				mean_y = PF_center_mean.block(1, i - num + 1, 1, num).sum() / num;

				std_x = sqrt((PF_center_mean.block(0, i - num + 1, 1, num).array() - mean_x).array().pow(2).sum() / (num));
				std_y = sqrt((PF_center_mean.block(1, i - num + 1, 1, num).array() - mean_y).array().pow(2).sum() / (num));
			}

			if ((i > 15) && ((location_state == 1) || ((std_x < 0.2) && (std_y < 0.2)))) // 定位稳定性判断
			{
				cout << "Location stable." << endl;
				location_state = 1;

				/*执行稳定状态逻辑*/
				tag_estimation_relative_x(i) = (PF_center_mean(0, i) - robot_xt[i]) * cos(-robot_tht[i]) - (PF_center_mean(1, i) - robot_yt[i]) * sin(-robot_tht[i]);
				tag_estimation_relative_y(i) = (PF_center_mean(0, i) - robot_xt[i]) * sin(-robot_tht[i]) + (PF_center_mean(1, i) - robot_yt[i]) * cos(-robot_tht[i]);

				// double tag_estimation_relative_x_0 = (tag_estimation_real_x(i) - robot_xt[i])*cos(-robot_tht[i]) - (tag_estimation_real_y(i) - robot_yt[i])*sin(-robot_tht[i]);
				// double tag_estimation_relative_y_0 = (tag_estimation_real_x(i) - robot_xt[i])*sin(-robot_tht[i]) + (tag_estimation_real_y(i) - robot_yt[i])*cos(-robot_tht[i]);
				cout << "机器人位姿为，x= " << robot_xt[i] << " y= " << robot_yt[i] << " th= " << robot_tht[i] << endl;
				cout << "标签绝对位置，x= " << PF_center_mean(0, i) << " y= " << PF_center_mean(1, i) << endl;
				cout << "标签相对位置，x= " << tag_estimation_relative_x(i) << "y=" << tag_estimation_relative_y(i) << endl;

				// 求解标签在机器人坐标系下的极坐标
				tag_distance(i) = sqrt(pow(tag_estimation_relative_x(i), 2) + pow(tag_estimation_relative_y(i), 2));
				tag_beta(i) = atan(abs(tag_estimation_relative_y(i) / tag_estimation_relative_x(i)));

				cout << "当前处于定位稳定阶段，i= " << i << endl;
				cout << "极距为" << tag_distance(i) << endl;
				cout << "极角为" << tag_beta(i) << endl;

				// 判定是否达到目标
				// if (abs(tag_estimation_relative_x(i) - tag_target_relative_x) > 10 || (abs(tag_target_relative_y - tag_estimation_relative_y(i)) > 10))
				if (((tag_distance(i) > tag_distance_threshold)) && (tag_estimation_relative_x(i) > tag_target_relative_x))
				{
					cout << "Controlling ..." << endl;
					// robot_vx(i)= pid_x_compute(robot_xt[i], PF_center_mean(0, i) );
					// //robot_vy(i)  = pid_y_compute(robot_yt[i],PF_center_mean(1, i) );
					// robot_w(i)  = pid_angle_compute(robot_tht[i], tag_beta(i));
					robot_vx(i) = 0.05;
					// robot_vy(i) = 0;
					robot_w(i) = 0;
					if (tag_beta(i) < -tag_beta_threshold)
					{
						robot_w(i) = 0.02;
					}
					if (tag_beta(i) > tag_beta_threshold)
					{
						robot_w(i) = -0.02;
					}
				}
				else
				{
					cout << "伺服控制成功 " << endl;
					robot_vx(i) = 0;
					// robot_vy(i) = 0;
					robot_w(i) = 0;
				}
			}
			else
			{
				cout << "Location unstable." << endl;
				location_state = 0;
				robot_vx(i) = robot_translational_vel0;
				// robot_vy(i) = 0;
				robot_w(i) = robot_rotational_vel0;
				step(i) = -1;
			}
		}
		else
		{
			robot_vx(i) = robot_translational_vel0;
			robot_vy(i) = 0;
			robot_w(i) = robot_rotational_vel0;
		}
	}
	cout << "Info: loop finished" << endl;
	return make_tuple(robot_vx(i), robot_w(i));
}

// tuple<double, double> Controller::getMotion(vector<TagData>* leftTagDataArray, vector<TagData>* rightTagDataArray, OdomData odom, int i)
// {
// 	// vector<double> dataVector;
// 	/* 记录某一时刻的机器人时间和系统时间戳
// 	const ArTime currntArTime;
// 	ftime(&currentSysTime);
// 	**********************************/
// 	cout << "Info: loop " << i << " started" << endl;

// 	srand((unsigned)time(NULL)); // 设定种子数--为了在c语言中生成随机数，配合rand使用

// 	/* 获取机器人的位置和姿态并进行单位换算 cm rad cm/s*/
// 	int odomNum = odom.robot_x.size();

// 	robot_xt[i] = odom.robot_x[odomNum - 1] ;
// 	robot_yt[i] = odom.robot_y[odomNum - 1] ; // m-->cm
// 	robot_tht[i] = odom.robot_th[odomNum - 1];
// 	// robot_vel[i] = odom.robot_vel[odomNum-1] * 100; // m/s-->cm/s

// 	std::cout << "Info: robot pose and velocity got" << std::endl;
// 	/* 获取当前读到的最新标签在数组中的下标(个数)，index */
// 	left_index_mapping[i] = leftTagDataArray.size() - 1;
// 	right_index_mapping[i] = rightTagDataArray.size() - 1;
// 	for (int p = (i > 3 ? left_index_mapping[i - 3] : 0); p <= left_index_mapping[i]; p++)
// 	{
// 		assignPoseForTagData(&(leftTagDataArray[p]), odom);
// 	}
// 	for (int p = (i > 3 ? right_index_mapping[i - 3] : 0); p <= right_index_mapping[i]; p++)
// 	{
// 		assignPoseForTagData(&(rightTagDataArray[p]), odom);
// 	}
// 	std::cout << "Info: assignPoseForTagData finished" << std::endl;
// 	/*由于硬件测量原因，需要先调整相位的值,进而对相位进行预处理*/

// 	if ((phase_flag_left == 0) && (left_index_mapping[i] >= 0))
// 	{
// 		// 左天线
// 		phase_flag_left = 1;
// 		phase_pre_left_antenna(phase_num_count_left) = leftTagDataArray[0].phase;
// 		phase_pre_left_antenna_2(phase_num_count_left) = (360 - phase_pre_left_antenna(phase_num_count_left)) / 180.0 * PI;
// 		phase_pos_left_antenna(phase_num_count_left) = phase_pre_left_antenna_2(phase_num_count_left);

// 		for (int j = 1; j <= left_index_mapping[i]; j++)
// 		{
// 			double diff = leftTagDataArray[j].phase - phase_pre_left_antenna(phase_num_count_left);
// 			if (diff > phase_half_fuzzy_index_lb && diff < phase_half_fuzzy_index_ub)
// 			{
// 				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray[j].phase - 180;
// 			}
// 			else if ((-diff) > phase_half_fuzzy_index_lb && (-diff) < phase_half_fuzzy_index_ub)
// 			{
// 				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray[j].phase + 180;
// 			}
// 			else
// 			{
// 				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray[j].phase;
// 			}
// 			phase_pre_left_antenna_2(phase_num_count_left) = (360 - phase_pre_left_antenna(phase_num_count_left)) / 180.0 * PI;
// 			int k = round((phase_pre_left_antenna_2(phase_num_count_left) - phase_pos_left_antenna(phase_num_count_left - 1)) / (2 * PI));
// 			phase_pos_left_antenna(phase_num_count_left) = phase_pre_left_antenna_2(phase_num_count_left) - 2 * PI * k;
// 		}
// 		cout << "左天线相位初始化完成 " << endl;
// 	}
// 	else if (phase_flag_left == 1)
// 	{
// 		// 左天线
// 		for (int j = left_index_mapping[i - 1] + 1; j <= left_index_mapping[i]; j++)
// 		{
// 			double diff = leftTagDataArray[j].phase - phase_pre_left_antenna(phase_num_count_left);
// 			if (diff > phase_half_fuzzy_index_lb && diff < phase_half_fuzzy_index_ub)
// 			{
// 				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray[j].phase - 180;
// 			}
// 			else if ((-diff) > phase_half_fuzzy_index_lb && (-diff) < phase_half_fuzzy_index_ub)
// 			{
// 				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray[j].phase + 180;
// 			}
// 			else
// 			{
// 				phase_pre_left_antenna(++phase_num_count_left) = leftTagDataArray[j].phase;
// 			}
// 			phase_pre_left_antenna_2(phase_num_count_left) = (360 - phase_pre_left_antenna(phase_num_count_left)) / 180.0 * PI;
// 			;
// 			int k = round((phase_pre_left_antenna_2(phase_num_count_left) - phase_pos_left_antenna(phase_num_count_left - 1)) / (2 * PI));
// 			phase_pos_left_antenna(phase_num_count_left) = phase_pre_left_antenna_2(phase_num_count_left) - 2 * PI * k;
// 		}
// 	}
// 	if ((phase_flag_right == 0) && (right_index_mapping[i] >= 0))
// 	{
// 		// 右天线
// 		phase_flag_right = 1;
// 		phase_pre_right_antenna(phase_num_count_right) = rightTagDataArray[0].phase;
// 		phase_pre_right_antenna_2(phase_num_count_right) = (360 - phase_pre_right_antenna(phase_num_count_right)) / 180.0 * PI;
// 		phase_pos_right_antenna(phase_num_count_right) = phase_pre_right_antenna_2(phase_num_count_right);

// 		for (int j = 1; j <= right_index_mapping[i]; j++)
// 		{
// 			double diff = rightTagDataArray[j].phase - phase_pre_right_antenna(phase_num_count_right);
// 			if (diff > phase_half_fuzzy_index_lb && diff < phase_half_fuzzy_index_ub)
// 			{
// 				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray[j].phase - 180;
// 			}
// 			else if ((-diff) > phase_half_fuzzy_index_lb && (-diff) < phase_half_fuzzy_index_ub)
// 			{
// 				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray[j].phase + 180;
// 			}
// 			else
// 			{
// 				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray[j].phase;
// 			}
// 			phase_pre_right_antenna_2(phase_num_count_right) = (360 - phase_pre_right_antenna(phase_num_count_right)) / 180.0 * PI;
// 			int k = round((phase_pre_right_antenna_2(phase_num_count_right) - phase_pos_right_antenna(phase_num_count_right - 1)) / (2 * PI));
// 			phase_pos_right_antenna(phase_num_count_right) = phase_pre_right_antenna_2(phase_num_count_right) - 2 * PI * k;
// 		}
// 		cout << "右天线相位初始化完成 " << endl;
// 	}
// 	else if (phase_flag_right == 1)
// 	{

// 		// 右天线
// 		for (int j = right_index_mapping[i - 1] + 1; j <= right_index_mapping[i]; j++)
// 		{
// 			double diff = rightTagDataArray[j].phase - phase_pre_right_antenna(phase_num_count_right);
// 			if (diff > phase_half_fuzzy_index_lb && diff < phase_half_fuzzy_index_ub)
// 			{
// 				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray[j].phase - 180;
// 			}
// 			else if ((-diff) > phase_half_fuzzy_index_lb && (-diff) < phase_half_fuzzy_index_ub)
// 			{
// 				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray[j].phase + 180;
// 			}
// 			else
// 			{
// 				phase_pre_right_antenna(++phase_num_count_right) = rightTagDataArray[j].phase;
// 			}
// 			phase_pre_right_antenna_2(phase_num_count_right) = (360 - phase_pre_right_antenna(phase_num_count_right)) / 180.0 * PI;
// 			int k = round((phase_pre_right_antenna_2(phase_num_count_right) - phase_pos_right_antenna(phase_num_count_right - 1)) / (2 * PI));
// 			phase_pos_right_antenna(phase_num_count_right) = phase_pre_right_antenna_2(phase_num_count_right) - 2 * PI * k;
// 		}
// 	}
// 	cout << "左天线相位当前数量: " << phase_num_count_left << endl;
// 	cout << "右天线相位当前数量: " << phase_num_count_right << endl;
// 	// for(int i=0;i<phase_num_count_left;i++)
// 	// {
// 	// 	cout<<"左天线相位: "<<phase_pos_left_antenna(i)<<endl;
// 	// }
// 	// for(int i=0;i<phase_num_count_right;i++)
// 	// {
// 	// 	cout<<"右天线相位: "<<phase_pos_right_antenna(i)<<endl;
// 	// }
// 	// cout<<"poseX:"<<robotPose.pose.pose.position.x<<endl;
// 	cout << "location_state:" << location_state << endl;
// 	if ((robot_xt[i] < sample_total_len) && (location_state == -1)) // 机器人的运动距离未达到定位的标准，且处于数据累计阶段
// 	{
// 		cout << "Particle Initial." << endl;
// 		PF_particle.row(0) = PF_scope_x_min + (PF_scope_x_max - PF_scope_x_min) / 2 * (MatrixXd::Random(1, PF_count).array() + 1);
// 		PF_particle.row(1) = PF_scope_y_min + (PF_scope_y_max - PF_scope_y_min) / 2 * (MatrixXd::Random(1, PF_count).array() + 1);
// 		// std::cout << "x:" << std::endl;
// 		// std::cout << PF_particle.row(0) << std::endl;
// 		// std::cout << "Y:" << std::endl;
// 		// std::cout << PF_particle.row(1) << std::endl;

// 		PF_center_mean(0, i) = 0; // 计算粒子的x轴坐标均值
// 		PF_center_mean(1, i) = 0; // 计算粒子的y轴坐标均值

// 		tag_estimation_real_x(i) = 0;
// 		tag_estimation_real_y(i) = 0;

// 		step(i) = -1;

// 		robot_vx(i) = robot_translational_vel0;
// 		robot_vy(i) = 0;
// 		robot_w(i) = robot_rotational_vel0;
// 	}
// 	else
// 	{
// 		cout << "Begin PF." << endl;
// 		random_device rd;
// 		mt19937 gen(rd());
// 		normal_distribution<double> normal(0, PF_Q); // 均值 和 标准差
// 		// normal(gen);

// 		for (int j = 0; j < PF_count; j++)
// 		{
// 			// double ran1 = normal(gen);
// 			PF_particle(0, j) = PF_particle(0, j) + normal(gen);
// 			// double ran2 = normal(gen);
// 			PF_particle(1, j) = PF_particle(1, j) + normal(gen);
// 		}
// 		//cout << "1" << endl;

// 		/*确定最后一个有效测量点的下标*/
// 		int leftEnd = left_index_mapping[i];
// 		int rightEnd = right_index_mapping[i];

// 		while (!leftTagDataArray[leftEnd].isAssignedPose)
// 		{
// 			leftEnd--;
// 		}
// 		while (!rightTagDataArray[rightEnd].isAssignedPose)
// 		{
// 			rightEnd--;
// 		}
// 		//cout << "2" << endl;
// 		/*确定距离 相位梯度 距离点的测量点的下标*/
// 		int leftStart = leftEnd;
// 		int rightStart = rightEnd;
// 		while (1)
// 		{
// 			double left_distance_interval = sqrt(pow(leftTagDataArray[leftEnd].x - leftTagDataArray[leftStart].x, 2) + pow(leftTagDataArray[leftEnd].y - leftTagDataArray[leftStart].y, 2));
// 			double right_distance_interval = sqrt(pow(rightTagDataArray[rightEnd].x - rightTagDataArray[rightStart].x, 2) + pow(rightTagDataArray[rightEnd].y - rightTagDataArray[rightStart].y, 2));
// 			if ((left_distance_interval > 6.0/100) || (right_distance_interval > 6.0/100))
// 			{
// 				break;
// 			}
// 			else
// 			{
// 				leftStart = leftStart - 1;
// 				rightStart = rightStart - 1;
// 			}
// 			if ((leftStart < 2) || (rightStart < 2))
// 			{
// 				break;
// 			}
// 			double left_phase_dis = sqrt(pow(leftTagDataArray[leftStart].x - leftTagDataArray[leftStart + 1].x, 2) + pow(leftTagDataArray[leftStart].y - leftTagDataArray[leftStart + 1].y, 2));
// 			double right_phase_dis = sqrt(pow(rightTagDataArray[rightStart].x - rightTagDataArray[rightStart + 1].x, 2) + pow(rightTagDataArray[rightStart].y - rightTagDataArray[rightStart + 1].y, 2));
// 			if ((left_phase_dis > 3) || (right_phase_dis > 3))
// 			{
// 				leftStart = leftStart + 1;
// 				rightStart = rightStart + 1;
// 				break;
// 			}
// 		}
// 		//cout << "3" << endl;
// 		double wave_length_var[16] = {32.5866, 32.5777, 32.5689, 32.56, 32.5512, 32.5424, 32.5336, 32.5247, 32.5159, 32.5071, 32.4983, 32.4895, 32.4807, 32.4719, 32.4631, 32.4544}; // 波长，cm
// 		/*左右天线前一时刻相位*/
// 		RowVectorXd PF_distance_left_antenna(PF_count);	 // 行向量
// 		RowVectorXd PF_distance_right_antenna(PF_count); // 行向量
// 		PF_distance_left_antenna = ((PF_particle.row(0).array() - leftTagDataArray[leftStart].x).array().pow(2) + (PF_particle.row(1).array() - leftTagDataArray[leftStart].y).array().pow(2)).cwiseSqrt().array() * 2;
// 		PF_distance_right_antenna = ((rightTagDataArray[rightStart].x - PF_particle.row(0).array()).array().pow(2) + (rightTagDataArray[rightStart].y - PF_particle.row(1).array()).array().pow(2)).cwiseSqrt().array() * 2;
// 		PF_particle.row(2) = ((PF_distance_left_antenna.array() / wave_length_var[0]) - (PF_distance_left_antenna.array() / wave_length_var[0]).floor()).array() * 2 * PI;
// 		PF_particle.row(5) = ((PF_distance_right_antenna.array() / wave_length_var[0]) - (PF_distance_right_antenna.array() / wave_length_var[0]).floor()).array() * 2 * PI;
// 		//cout << "3.1" << endl;
// 		///*左右天线当前时刻相位*/
// 		PF_distance_left_antenna = ((leftTagDataArray[leftEnd].x - PF_particle.row(0).array()).array().pow(2) + (leftTagDataArray[leftEnd].y - PF_particle.row(1).array()).array().pow(2)).cwiseSqrt().array() * 2;
// 		PF_distance_right_antenna = ((rightTagDataArray[rightEnd].x - PF_particle.row(0).array()).array().pow(2) + (rightTagDataArray[rightEnd].y - PF_particle.row(1).array()).array().pow(2)).cwiseSqrt().array() * 2;
// 		PF_particle.row(3) = ((PF_distance_left_antenna.array() / wave_length_var[0]) - (PF_distance_left_antenna.array() / wave_length_var[0]).floor()) * 2 * PI;
// 		PF_particle.row(6) = ((PF_distance_right_antenna.array() / wave_length_var[0]) - (PF_distance_right_antenna.array() / wave_length_var[0]).floor()) * 2 * PI;
// 		//cout << "3.2" << endl;
// 		///*左右天线当前相位梯度*/
// 		RowVectorXd k(PF_count);
// 		k = ((PF_particle.row(3) - PF_particle.row(2)).array() / (2 * PI)).round();
// 		PF_particle.row(4) = PF_particle.row(3).array() - 2 * PI * k.array();
// 		PF_particle.row(4) = PF_particle.row(4).array() - PF_particle.row(2).array();
// 		k = ((PF_particle.row(6) - PF_particle.row(5)).array() / (2 * PI)).round();
// 		PF_particle.row(7) = PF_particle.row(6).array() - 2 * PI * k.array();
// 		PF_particle.row(7) = PF_particle.row(7).array() - PF_particle.row(5).array();
// 		//cout << "3.3" << endl;
// 		/*实际观测相位梯度*/
// 		double PF_observe_left = phase_pos_left_antenna(leftEnd) - phase_pos_left_antenna(leftStart);
// 		double PF_observe_right = phase_pos_right_antenna(rightEnd) - phase_pos_right_antenna(rightStart);

// 		/*粒子权重评估*/
// 		PF_distance.row(0) = PF_particle.row(4).array() - PF_observe_left;
// 		PF_w.row(0) = (1 / sqrt(2 * PF_R) / sqrt(2 * PI)) * (-PF_distance.row(0).array().pow(2).array() / (4 * PF_R)).array().exp(); // 求权重

// 		PF_distance.row(1) = PF_particle.row(7).array() - PF_observe_right;
// 		PF_w.row(1) = (1 / sqrt(2 * PF_R) / sqrt(2 * PI)) * (-PF_distance.row(1).array().pow(2).array() / (4 * PF_R)).array().exp(); // 求权重

// 		PF_w.row(2) = PF_w.row(0).array() * PF_w.row(1).array(); // 综合权重
// 		//cout << "4" << endl;
// 		/*重采样*/
// 		MatrixXd PF_particle_new(8, PF_count);
// 		int index = (((double)rand()) / RAND_MAX) * (PF_count - 1) + 0; // 生成0 - （PF_count-1） 的随机数
// 		double beta = 0;
// 		double mw = PF_w.row(2).maxCoeff();
// 		for (int j = 0; j < PF_count; j++)
// 		{
// 			beta = beta + 2 * mw * (((double)rand()) / RAND_MAX);
// 			while (beta > PF_w(2, index))
// 			{
// 				beta = beta - PF_w(2, index);
// 				index = (index + 1) % PF_count;
// 			}
// 			PF_particle_new.col(j) = PF_particle.col(index);
// 		}
// 		PF_particle = PF_particle_new;
// 		//cout << "5" << endl;
// 		///*定位结果*/
// 		PF_center_mean(0, i) = PF_particle.block(0, 0, 1, PF_count).sum() / PF_count; // 计算粒子的x轴坐标均值
// 		PF_center_mean(1, i) = PF_particle.block(1, 0, 1, PF_count).sum() / PF_count; // 计算粒子的y轴坐标均值

// 		int num = 5;

// 		if (i >= num - 1)
// 		{
// 			double mean_x, mean_y, std_x, std_y;
// 			if (location_state != 1)
// 			{
// 				mean_x = PF_center_mean.block(0, i - num + 1, 1, num).sum() / num;
// 				mean_y = PF_center_mean.block(1, i - num + 1, 1, num).sum() / num;

// 				std_x = sqrt((PF_center_mean.block(0, i - num + 1, 1, num).array() - mean_x).array().pow(2).sum() / (num));
// 				std_y = sqrt((PF_center_mean.block(1, i - num + 1, 1, num).array() - mean_y).array().pow(2).sum() / (num));
// 			}

// 			if ((i > 15) && ((location_state == 1) || ((std_x < 0.2) && (std_y < 0.2)))) // 定位稳定性判断
// 			{
// 				cout << "Location stable." << endl;
// 				location_state = 1;

// 				/*执行稳定状态逻辑*/
// 				tag_estimation_relative_x(i) = (PF_center_mean(0, i) - robot_xt[i]) * cos(-robot_tht[i]) - (PF_center_mean(1, i) - robot_yt[i]) * sin(-robot_tht[i]);
// 				tag_estimation_relative_y(i) = (PF_center_mean(0, i) - robot_xt[i]) * sin(-robot_tht[i]) + (PF_center_mean(1, i) - robot_yt[i]) * cos(-robot_tht[i]);

// 				// double tag_estimation_relative_x_0 = (tag_estimation_real_x(i) - robot_xt[i])*cos(-robot_tht[i]) - (tag_estimation_real_y(i) - robot_yt[i])*sin(-robot_tht[i]);
// 				// double tag_estimation_relative_y_0 = (tag_estimation_real_x(i) - robot_xt[i])*sin(-robot_tht[i]) + (tag_estimation_real_y(i) - robot_yt[i])*cos(-robot_tht[i]);
// 				cout<<"机器人位姿为x= "<<robot_xt[i]<<" y= "<<robot_yt[i]<<" th= "<<robot_tht[i]<<endl;
// 				cout<<"标签绝对位置，x="<<PF_center_mean(0, i)<<"y="<<PF_center_mean(1, i)<<endl;
// 				cout << "标签相对位置，x= " << tag_estimation_relative_x(i) << "y=" << tag_estimation_relative_y(i) << endl;

// 				// 求解标签在机器人坐标系下的极坐标
// 				tag_distance(i) = sqrt(pow(tag_estimation_relative_x(i), 2) + pow(tag_estimation_relative_y(i), 2));
// 				tag_beta(i) = atan(abs(tag_estimation_relative_y(i) / tag_estimation_relative_x(i)));

// 				cout << "当前处于定位稳定阶段，i= " << i << endl;
// 				cout<<"极角为"<<tag_beta(i)<<endl;

// 				// 判定是否达到目标
// 				// if (abs(tag_estimation_relative_x(i) - tag_target_relative_x) > 10 || (abs(tag_target_relative_y - tag_estimation_relative_y(i)) > 10))
// 				if (((tag_distance(i) > tag_distance_threshold)) && (tag_estimation_relative_x(i) > tag_target_relative_x))
// 				{
// 					cout << "Controlling ..." << endl;
// 					// robot_vx(i)= pid_x_compute(robot_xt[i], PF_center_mean(0, i) );
// 					// //robot_vy(i)  = pid_y_compute(robot_yt[i],PF_center_mean(1, i) );
// 					// robot_w(i)  = pid_angle_compute(robot_tht[i], tag_beta(i));
// 					robot_vx(i) = 0.05;
// 					// robot_vy(i) = 0;
// 					robot_w(i) = 0;
// 					if (tag_beta(i) > tag_beta_threshold)
// 					{
// 						robot_w(i) = 0.05;
// 					}
// 				}
// 				else
// 				{
// 					cout << "伺服控制成功 " << endl;
// 					robot_vx(i) = 0;
// 					// robot_vy(i) = 0;
// 					robot_w(i) = 0;
// 				}
// 			}
// 			else
// 			{
// 				cout << "Location unstable." << endl;
// 				location_state = 0;
// 				robot_vx(i) = robot_translational_vel0;
// 				// robot_vy(i) = 0;
// 				robot_w(i) = robot_rotational_vel0;
// 				step(i) = -1;
// 			}
// 		}
// 		else
// 		{
// 			robot_vx(i) = robot_translational_vel0;
// 			robot_vy(i) = 0;
// 			robot_w(i) = robot_rotational_vel0;
// 		}
// 	}
// 	cout << "Info: loop finished" << endl;
// 	return make_tuple(robot_vx(i), robot_w(i));
// }