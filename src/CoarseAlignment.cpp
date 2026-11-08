//惯性导航初始粗对准
#include "IMU.h"
using namespace std;
using namespace Eigen;
double GetDiffTime(GPSTIME& t1,GPSTIME& t2){
    return (t2.Week - t1.Week) * 604800 + (t2.SOW - t1.SOW);
}
void CalculateCbn(AlignModule& align){
    // 将加速度计和陀螺仪数据转换为向量形式
	Vector3d gb = Vector3d::Zero(3, 1);	// 比力
	Vector3d wb = Vector3d::Zero(3, 1);	// 角速度
    Vector3d v = Vector3d::Zero(3, 1);  // 计算叉乘辅助向量

    for (int i = 0; i < 3; i++)
	{
		gb(i) = -align.fmean[i];
		wb(i) = align.anglevelocitymean[i];
	}
	v = gb.cross(wb);	// 叉乘计算
	
	// 定义导航坐标系下的参考向量
	Vector3d gn(0.0, 0.0, gp);	// 重力加速度向量
	Vector3d wn(OMEGA* rad2deg* cos(Latitude), 0.0, -OMEGA* rad2deg * sin(Latitude));	// 地球自转角速度向量

	// 构造载体坐标系下的正交基
	Vector3d wg = gb.normalized();	// 单位化比力向量
	Vector3d ww = v.normalized();
	Vector3d wgw = (v.cross(gb)).normalized();

	Vector3d vg = gn.normalized();	// 单位化重力向量
	Vector3d vw = (gn.cross(wn)).normalized();
	Vector3d vgw = ((gn.cross(wn)).cross(gn)).normalized();

	// 构造矩阵并计算姿态矩阵
	MatrixXd V = MatrixXd(3, 3); V.setZero();	// 导航系正交基矩阵
	V.col(0) = vg; V.col(1) = vw; V.col(2) = vgw;
	MatrixXd W = MatrixXd(3, 3); W.setZero();	// 载体系正交基矩阵
	W.col(0) = wg; W.col(1) = ww; W.col(2) = wgw;

	align.Cbn = V * W.transpose();	// 计算姿态矩阵

	align.AttitudeAngle();	// 提取姿态角
}

//先对每个历元进行粗对准
void EpochAlign(ifstream& FilePath,ASCData data,AlignModule& now,AlignModule& total,ofstream& alignfile){
    data.ft = ALIGNMENT;
    //初始化加速度计和陀螺仪数据的均值
    double accemean[3] = {0.0, 0.0, 0.0};
    double anglevelmean[3] = {0.0, 0.0, 0.0};
    double epochnum = 0.0;
    double RMSE[3] = {0.0, 0.0, 0.0};

    while(ReadFile(FilePath,data)){
        now.time = data.time;

        now.fmean[0] = data.acce.rax;
        now.fmean[1] = data.acce.ray;
        now.fmean[2] = data.acce.raz;

        now.anglevelocitymean[0] = data.gyro.rgx;
        now.anglevelocitymean[1] = data.gyro.rgy;
        now.anglevelocitymean[2] = data.gyro.rgz;
        epochnum++;
        //计算姿态矩阵
        CalculateCbn(now);
        RMSE[0] += pow(now.yaw - total.yaw, 2);
        RMSE[1] += pow(now.pitch - total.pitch, 2);
        RMSE[2] += pow(now.roll - total.roll, 2);

        alignfile << fixed << setprecision(12);
        alignfile << now.time.SOW << ",";
        alignfile << now.yaw << "," << now.pitch << "," << now.roll << endl;
    }
    for (int i = 0; i < 3;i++){
        RMSE[i] = sqrt(RMSE[i] / epochnum);
    }
}

//每秒进行粗对准
void SecondAlign(ifstream& FilePath,ASCData data,AlignModule& now,AlignModule& total,ofstream& alignfile){
    data.ft = ALIGNMENT;
    double accemean[3] = {0.0, 0.0, 0.0};
    double anglevelmean[3] = {0.0, 0.0, 0.0};
    double epochnum = 0.0;
    double RMSE[3] = {0.0, 0.0, 0.0};
    double dt = 0.0;
    double totaltime = 0.0;
    int T = 0;

    while(ReadFile(FilePath,data)){
        now.time = data.time;
        if(epochnum!=0){
            dt = GetDiffTime(now.prevtime, now.time);
        }
        totaltime += dt;
        if(fabs(totaltime-1.0)>1e-3){
            accemean[0] += data.acce.rax;
            accemean[1] += data.acce.ray;
            accemean[2] += data.acce.raz;

            anglevelmean[0] += data.gyro.rgx;
            anglevelmean[1] += data.gyro.rgy;
            anglevelmean[2] += data.gyro.rgz;

            epochnum++;
        }
        else if(fabs(totaltime-1.0)<1e-3){
            for (int i = 0; i < 3;i++){
                now.fmean[i] = accemean[i] / epochnum;
                now.anglevelocitymean[i] = anglevelmean[i] / epochnum;
            }
            CalculateCbn(now);
            RMSE[0] += pow(now.yaw - total.yaw, 2);
            RMSE[1] += pow(now.pitch - total.pitch, 2);
            RMSE[2] += pow(now.roll - total.roll, 2);

            T++;

            alignfile << T << ",";
            alignfile << fixed << setprecision(12);
            alignfile << now.yaw << "," << now.pitch << "," << now.roll << endl;

            for (int i = 0; i < 3;i++){
                accemean[i] = 0.0;
                anglevelmean[i] = 0.0;
            }
            totaltime = 0.0;
            epochnum = 1e-13;
        }
        now.prevtime = now.time;
    }
    for (int i = 0; i < 3;i++){
        RMSE[i] = sqrt(RMSE[i] / T);
    }
}

void AllAlign(ifstream& FilePath, ASCData& data, AlignModule& align, ofstream& alignres){
    data.ft = ALIGNMENT;
	// 初始化加速度计和陀螺仪数据的均值
	double accemean[3] = { 0.0,0.0,0.0 };	// [0.0, 0.0, 0.0]
	double anglevelmean[3] = { 0.0,0.0,0.0 };
	double epochnum = 0.0;

	// 循环读取数据
	while (ReadFile(FilePath, data))
	{
		accemean[0] += data.acce.rax;
		accemean[1] += data.acce.ray;
		accemean[2] += data.acce.raz;

		anglevelmean[0] += data.gyro.rgx;	// 角速度
		anglevelmean[1] += data.gyro.rgy;	// 角速度
		anglevelmean[2] += data.gyro.rgz;	// 角速度

		epochnum += 1.0;
	}
	// 求均值
	for (int i = 0; i < 3; i++)
	{
		align.fmean[i] = accemean[i] / epochnum;
		align.anglevelocitymean[i] = anglevelmean[i] / epochnum;
	}

	// 计算姿态矩阵
	CalculateCbn(align);

	alignres << fixed << setprecision(12);
	alignres << align.yaw << "," << align.pitch << "," << align.roll << endl;

	cout << "\nWhole-dataset coarse alignment completed. " << "The attitude results have been compared with the reference.\n";
}

/* 计算时间噪声特性 */
void CalTimeNoise(ifstream& FilePath, ASCData& data, ofstream& alignres){
    data.ft = ALIGNMENT;
	alignres << "%Bias(deg): " << fixed << setprecision(12) <<  ((0.05 / 3600.0) / (OMEGA * cos(Latitude))) << endl;
	alignres << "%Time(s),WhiteNoise(deg):\n";

	// 初始化加速度计和陀螺仪数据的均值
	double t0 = 0.0, totalt = 0;
	double epochnum = 0.0;

	// 循环读取数据
	while (ReadFile(FilePath, data))
	{
		if (epochnum == 0) { 
            t0 = data.time.Week * 604800 + data.time.SOW;
        }	// 获取初始时间
		epochnum++;
		totalt = data.time.Week * 604800 + data.time.SOW - t0;
		if (epochnum == 1){
            continue;
        }	// 跳过totalt为0的情况

		alignres << fixed << setprecision(4);
		alignres << totalt << ",";
		alignres << fixed << setprecision(12);
		alignres <<  ((0.005 / 3600.0) / ((OMEGA * cos(Latitude)) * sqrt(totalt))) << endl;
	}

	cout << "\nTime-noise analysis completed. " << "The results have been compared with the reference.\n";
}