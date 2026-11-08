#include "IMU.h"

using namespace std;
// 地球自转
// Vector3d EarthRotationProjection(const double omega,const double latitudeRad){
//     Vector3d result;
//     result << omega * cos(latitudeRad),
//               0.0,
//              -omega * sin(latitudeRad);
//     return result;
// }
double EarthRotationProjection(const double omega,const double latitudeRad){
    return -omega * sin(latitudeRad);
}
// Z=X*H的最小二乘模型求解
MatrixXd LeastSquare(const MatrixXd& Z,const MatrixXd& H){
    return Z * H.transpose() * (H * H.transpose()).inverse();
} 

//计算加速度均值
void CalculateFMean(ifstream& FilePath,ASCData& data,int pos,AcceCalibrationModule& cali){
	data.ft = CALIBRATION;
	double accemean[3] = {0.0, 0.0, 0.0};
    double epochnum = 0.0;
    while(ReadFile(FilePath,data)){
        accemean[0] += data.acce.rax;
        accemean[1] += data.acce.ray;
        accemean[2] += data.acce.raz;
        epochnum++;
    }
    for (int i = 0; i < 3;i++){
        cali.fmean(i,pos) = accemean[i] / epochnum;
    }
}



// 加速度计六位置法标定
void AcceCalibration(AcceCalibrationModule& cali,ofstream& resfile){
    // 真实比力
    MatrixXd Freal = MatrixXd(4, 6);
    Freal << gp, -gp, 0, 0, 0, 0,
        0, 0, gp, -gp, 0, 0,
        0, 0, 0, 0, gp, -gp,
        1, 1, 1, 1, 1, 1;
    
    // 实测比力cali.fmean
    //计算标定矩阵
    cali.Caliresult = cali.fmean * Freal.transpose() * (Freal * Freal.transpose()).inverse();
    cali.AcceBias();
    cali.AcceScale();
    cali.AcceCross();

    resfile << fixed << setprecision(12);
    resfile << "%Bias(m/s2)\t" << cali.bias[0] << ',' << cali.bias[1] << ',' << cali.bias[2] << endl;
    resfile << "%Scale\t" << cali.scale[0] << ',' << cali.scale[1] << ',' << cali.scale[2] << endl;
    resfile << "%Cross\t" << cali.cross(0, 0) << '\t' << cali.cross(0, 1) << '\t' << cali.cross(0, 2) << '\n'
            << cali.cross(1, 0) << '\t' << cali.cross(1, 1) << '\t' << cali.cross(1, 2) << '\n'
            << cali.cross(2, 0) << '\t' << cali.cross(2, 1) << '\t' << cali.cross(2, 2) << endl;

    cout << "Accelerometer calibration completed." << endl;
}

/* 利用输出的加速度计 对原始加速度计测量前后补偿 */
void CompensateAcc(ifstream& FilePath, ASCData& data, AcceCalibrationModule& cali, ofstream& resfile)
{
	data.ft = CALIBRATION;
	resfile << "%Time Compensation Raw(m/s^2):\n";

	// 逐个读取文件
	while (ReadFile(FilePath, data))
	{
		Vector3d Com = Vector3d::Zero();	// 补偿结果
		Vector3d Raw = Vector3d::Zero();	// 原始数据

		MatrixXd M = MatrixXd(3, 3); M.setZero();	// 比例零偏和交叉耦合
		M = cali.Caliresult.block<3, 3>(0, 0);	// 比例
		Vector3d b = Vector3d::Zero();	// 零偏向量
		b = cali.Caliresult.col(3);  // 零偏

		// 获取原始数据
		Raw(0) = data.acce.rax;
		Raw(1) = data.acce.ray;
		Raw(2) = data.acce.raz;

		// 进行补偿
		Com = M.inverse() * (Raw - b);

		resfile << fixed << setprecision(2);
		resfile << data.time.SOW << ",";

		resfile << fixed << setprecision(12);
		resfile << Com(0) << "," << Com(1) << "," << Com(2) << ","
			<< Raw(0) << "," << Raw(1) << "," << Raw(2) << endl;
	}
	cout << "\nAccelerometer compensation completed. "
     << "Raw and compensated acceleration data have been saved.\n";
}

// 计算旋转角度
void CalculateRotAngle(ifstream& FilePath,ASCData data,int mode,int Axis,GyroCalibrationModule& gyro,ofstream& gyroresfile){
    data.ft = CALIBRATION;
	gyroresfile << "%Time X Y Z(Deg):\n";

	// 初始化角度
	double angle[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;	// 历元数
	double totalT = 0.0;	// 积分时间
	bool pprev = false, prev = false;
	bool normal = false;

	// 逐个读取文件，进行积分
	while (ReadFile(FilePath, data))
	{
		// 是否开始积分标志
		bool cur = false;

		// 获取时间间隔
		double dt = 0.0, dt_prev = 0.0;
		if (epochnum != 0){
            dt = (data.time.Week - gyro.prevtime.Week) * 604800 + data.time.SOW - gyro.prevtime.SOW;
        }

		// 梯形法进行角速度积分
		double X = 0.0, Y = 0.0, Z = 0.0;
		if (epochnum > 0 && dt > 0.0)
		{
			X = 0.5 * (data.gyro.rgx + gyro.prevgyro.rgx) * dt * rad2deg;
			Y = 0.5 * (data.gyro.rgy + gyro.prevgyro.rgy) * dt * rad2deg;
			Z = 0.5 * (data.gyro.rgz + gyro.prevgyro.rgz) * dt * rad2deg;

			gyroresfile << fixed << setprecision(12);
			gyroresfile << data.time.SOW << "," << X << "," << Y << "," << Z << endl;

			// 获取积分信息
			double add = 0.0;
			switch (Axis)
			{
			case 0: { add = X; break; }
			case 1: { add = Y; break; }
			case 2: { add = Z; break; }
			}
			// 判断是否达到阈值
			if (fabs(add) >= Increment) { cur = true; }	// 若超过阈值则开始积分

			// 进行积分
			if (cur == true && fabs(totalT - Datatime) > 1e-3)
			{
				angle[0] += X;
				angle[1] += Y;
				angle[2] += Z;
				totalT += dt;
				normal = true;
			}
			// 若超过阈值之后但是未到达指定积分时间readtime，继续进行积分
			else if (normal == true && fabs(totalT - Datatime) > 1e-3)
			{
				//cout << Rawdata.time.Second << endl;
				angle[0] += X;
				angle[1] += Y;
				angle[2] += Z;
				totalT += dt;
			}

			if (fabs(totalT - Datatime) < 1e-3)
			{
				int j = 0;
			}
		}
		dt_prev = dt;

		// 更新角速度，时间
		gyro.prevgyro = data.gyro;
		gyro.prevtime = data.time;

		epochnum += 1.0;
	}

	// 存储结果结构体
	if (mode == 0)
	{
		gyro.posT = totalT;
		for (int i = 0; i < 3; i++)
		{
			gyro.posangle.rgx = angle[0];
			gyro.posangle.rgy = angle[1];
			gyro.posangle.rgz = angle[2];
		}
	}
	else if (mode == 1)
	{
		gyro.negT = totalT;
		for (int i = 0; i < 3; i++)
		{
			gyro.negangle.rgx = angle[0];
			gyro.negangle.rgy = angle[1];
			gyro.negangle.rgz = angle[2];
		}
	}
}


// 陀螺误差标定
void GyroCalibration(int Axis,GyroCalibrationModule& cali,int mode){
    double totalT = (cali.negT + cali.posT) / 2.0;	// 积分时间为正转和反转的平均

	// 零偏
	switch (Axis)
	{
	case 0:
		if (mode == 0) {
			cali.bias[0] = (cali.posangle.rgx + cali.negangle.rgx) / (2.0 * totalT) - OMEGA*rad2deg * sin(Latitude);	// 零偏
			cali.scale[0] = abs((cali.posangle.rgx - cali.negangle.rgx)) / (2.0 * Circle) - 1.0;		// 比例因子
		}
		else if (mode == 1){
            cali.bias[0] = (cali.posangle.rgx + cali.negangle.rgx) / (2.0);
        }
		break;
	case 1:
		if (mode == 0) {
			cali.bias[1] = (cali.posangle.rgy + cali.negangle.rgy) / (2.0 * totalT) - OMEGA*rad2deg * sin(Latitude);
			cali.scale[1] = abs((cali.posangle.rgy - cali.negangle.rgy)) / (2.0 * Circle) - 1.0;
		}
		else if (mode == 1){
            cali.bias[1] = (cali.posangle.rgy + cali.negangle.rgy) / (2.0);
        }
		break;
	case 2:
		if (mode == 0) {
			cali.bias[2] = (cali.posangle.rgz + cali.negangle.rgz) / (2.0 * totalT) - OMEGA*rad2deg * sin(Latitude);
			cali.scale[2] = abs((cali.posangle.rgz - cali.negangle.rgz)) / (2.0 * Circle) - 1.0;
		}
		else if (mode == 1){
            cali.bias[2] = (cali.posangle.rgz + cali.negangle.rgz) / (2.0);
        }
		break;
	}
	for (int i = 0; i < 3; i++) { 
        cali.bias[i] = cali.bias[i] * rad2deg * 3600.0;
    }	// 度每秒转度每小时
}

void CalculateGyroBias(ifstream& FilePath, ASCData& data, GyroCalibrationModule& cali, int mode)
{
	data.ft = CALIBRATION;
	// 初始化三个方向上的平均值
	double gyrmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;

	// 逐个读取文件
	while (ReadFile(FilePath, data))
	{
		gyrmean[0] += data.gyro.rgx;
		gyrmean[1] += data.gyro.rgy;
		gyrmean[2] += data.gyro.rgz;

		epochnum ++;
	}
	if (mode == 0)
	{
		cali.posangle.rgx = gyrmean[0] / epochnum;
		cali.posangle.rgy = gyrmean[1] / epochnum;
		cali.posangle.rgz = gyrmean[2] / epochnum;
	}
	if (mode == 1)
	{
		cali.posangle.rgx = gyrmean[0] / epochnum;
		cali.posangle.rgy = gyrmean[1] / epochnum;
		cali.posangle.rgz = gyrmean[2] / epochnum;
	}
}

/* 利用输出结果 输出原始与补偿后的数据 */
void CompensateGyro(ifstream& FilePath, ASCData& data, int mode, GyroCalibrationModule& cali, ofstream& gyroresFile)
{
	data.ft = CALIBRATION;
	gyroresFile << "%Time Compensation_XYZ Raw_XYZ(Rad/s):\n";
	// 逐个读取文件
	while (ReadFile(FilePath, data))
	{
		Vector3d Com = Vector3d::Zero();	// 补偿结果
		Vector3d Raw = Vector3d::Zero();	// 原始数据

		MatrixXd M = MatrixXd(3, 3); M.setZero();	// 比例零偏和交叉耦合
		for (int i = 0; i < 3; i++)M(i, i) = cali.scale[i] + 1.0;	// 获取比例因子


		Vector3d b = Vector3d::Zero();	// 零偏向量
		for (int i = 0; i < 3; i++)b(i) = cali.bias[i] * deg2rad / 3600.0;	// 获取零偏(度/h->rad/s)

		// 获取原始数据
		Raw(0) = data.gyro.rgx;
		Raw(1) = data.gyro.rgy;
		Raw(2) = data.gyro.rgz;

		// 进行补偿
		Com = M.inverse() * (Raw - b);

		gyroresFile << fixed << setprecision(2);
		gyroresFile << data.time.SOW<< ",";
		gyroresFile << fixed << setprecision(12);

		switch (mode)
		{
		case 0:
			gyroresFile << Com(0) - OMEGA* sin(Latitude)  << "," << Raw(0) << endl;
			break;
		case 1:
			gyroresFile << Com(1) - OMEGA* sin(Latitude) << "," << Raw(1) << endl;
			break;
		case 2:
			gyroresFile << Com(2) - OMEGA* sin(Latitude) << "," << Raw(2) << endl;
			break;
		}
	}
	cout << "Gyroscope calibration completed. Result files have been generated.\n";
}
