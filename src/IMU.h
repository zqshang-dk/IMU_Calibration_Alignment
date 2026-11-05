#pragma once
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <array>
#include <filesystem>
#include <map>

using namespace std;
using namespace Eigen;

// UnitConvention
#define PI               3.14159265358979323846
#define deg2rad          (PI/180.0)
#define rad2deg          (180.0/PI)
#define OMEGA            7.292115*1e-5     //地球自转角速度
#define Latitude         (30.531651244 * deg2rad)
#define Circle           360.0
#define gp               9.7936174         // 当地的重力加速度，M/S2
// 本批数据：标定为 NovAtel-FSAS，对准为 NovAtel-100C，均为 200 Hz。
// 保留原接口名称；计数系数统一为 (m/s)/count 和 rad/count。
constexpr double FNSC = 200.0;
constexpr double FXWG = 200.0;
constexpr double AS_NSC = 2.0e-8;
constexpr double GS_NSC = 1.0e-9;
constexpr double AS_XWG = 0.05 / 32768.0;
constexpr double GS_XWG = (0.1 / (3600.0 * 256.0)) * deg2rad;
constexpr double StaticTrimSeconds = 5.0;
//GPStime的结构体
struct GPSTIME{
	int Week;
	double SOW;
	GPSTIME(){
		Week = 0;
		SOW = 0.0;
	}
};
// 三轴加速度计
union Acceleration {
	double acce[3];
	struct {
		double rax;
		double ray;
		double raz;
	};
	Acceleration(){
		for (int i = 0; i < 3;i++){
			acce[i] = 0.0;
		}
		rax = ray = raz = 0.0;
	}
};

// 三轴陀螺仪
union GyroSensor {
	double gyro[3];
	struct {
		double rgx;
		double rgy;
		double rgz;
	};
	GyroSensor(){
		for (int i = 0; i < 3;i++){
			gyro[i] = 0.0;
		}
		rgx = rgy = rgz = 0.0;
	}
};
enum FileType{CALIBRATION,ALIGNMENT};

// ASC的数据格式
struct ASCData{
	FileType ft;
	// int IMUInfo;
    // int IMUType;
	
	// string messageType;    //区分 RAWIMUSA 和 RAWIMUXA

	GPSTIME time;
	// string IMUStatus;      //状态字

    Acceleration acce;        // 量程换算后的未标定三轴比力，单位 m/s^2
	GyroSensor gyro;        // 量程换算后的未标定三轴角速度，单位 rad/s

    // double deltaV[3];       // 量程换算后的速度增量，m/s
    // double deltaTheta[3];   // 量程换算后的角增量，rad

    // double force[3];        // 加速度计标定后的比力，m/s^2
    // double omega[3];        // 陀螺仪标定后的角速度，rad/s
	ASCData(){
		ft = CALIBRATION;
		time.Week = 0;
		time.SOW = 0.0;
	}
};

// 误差标定的结构体
struct AcceCalibrationModule
{
    // 六个位置的三轴平均比力，单位：m/s²
	MatrixXd fmean;           //比力平均值

	double bias[3];// 加速度计零偏
	double scale[3];// 三轴比例因子误差
	Matrix3d cross; //交轴耦合
	MatrixXd Caliresult;           //标定结果的M矩阵
	AcceCalibrationModule(){
		for (int i = 0; i < 3;i++){
			bias[i] = scale[i] = 0.0;
		}
		fmean = MatrixXd(3, 6);
		fmean.setZero();
		cross.setZero();
		Caliresult = MatrixXd(3, 4);
		Caliresult.setZero();
	}
	void AcceBias(){
		for(int i = 0; i < 3;i++){
			bias[i] = Caliresult(i, 3);
		}
	}
	void AcceScale(){
		for (int i = 0; i < 3;i++){
			scale[i] = Caliresult(i, i) - 1;
		}
	}
	void AcceCross(){
		cross(0, 1) = Caliresult(0, 1);
		cross(0, 2) = Caliresult(0, 2);
		cross(1, 0) = Caliresult(1, 0);
		cross(1, 2) = Caliresult(1, 2);
		cross(2, 0) = Caliresult(2, 0);
		cross(2, 1) = Caliresult(2, 1);
	}
};

struct GyroCalibrationModule{
	GPSTIME prevtime;
	GyroSensor prevgyro;
	GyroSensor posangle;
	GyroSensor negangle;
    GyroSensor staticUp;       // rad/s，不与正反转积分角混用
    GyroSensor staticDown;
    bool hasStaticUp = false;
    bool hasStaticDown = false;
    int verticalSign = -1;     // 本批 *_down_*：敏感轴朝下
	double posT;
	double negT;
	double bias[3];             // 内部统一 rad/s；输出时才转 deg/h
	double scale[3];
	GyroCalibrationModule(){
		posT = 0.0;
		negT = 0.0;
		for (int i = 0; i < 3;i++){
			bias[i] = 0.0;
			scale[i] = 0.0;
		}
	}
};

struct AlignModule{
	GPSTIME time;
	GPSTIME prevtime;
	double fmean[3];
	double anglevelocitymean[3];
	double yaw, pitch, roll;// 航向角、俯仰角、横滚角
	double Accebias, Accenoise;//加速度计的零偏与噪声
	MatrixXd Cbn; //姿态转换矩阵

	AlignModule(){
		yaw = pitch = roll = 0.0;
		Accebias = Accenoise = 0.0;
		for (int i = 0; i < 3;i++){
			fmean[i] = anglevelocitymean[i] = 0.0;
		}
		Cbn = MatrixXd(3, 3);
		Cbn.setZero();
	}
	void AttitudeAngle(){
		double C31 = Cbn(2, 0);
		double C32 = Cbn(2, 1);
		double C33 = Cbn(2, 2);
		double C21 = Cbn(1, 0);
		double C11 = Cbn(0, 0);

		yaw = atan2(C21, C11);
		pitch = atan2(-C31, hypot(C32, C33));
		roll = atan2(C32, C33);
	}
};


// 程序配置
struct AppConfig
{
    int mode = -1;

    // 加速度计标定
    string accCalibrationFiles[6];
    string accCompensationFiles[6];

    string accResultFile;
    string accCompensationResultFiles[6];

    // 陀螺仪标定
    string gyroRotationFiles[6];
    string gyroCompensationFiles[6];
    string gyroStaticFiles[6];

    string gyroResultFile;
    string gyroAngleResultFiles[6];
    string gyroCompensationResultFiles[6];

    // 初始对准
    string alignmentInputFile;

    string alignmentWholeResultFile;
    string alignmentSecondResultFile;
    string alignmentEpochResultFile;
    string alignmentNoiseResultFile;
};

// 读取配置文件
bool LoadConfig(const string& filename, AppConfig& config);
bool ReadFile(ifstream &file, ASCData &data);
vector<ASCData> ReadRecords(ifstream& file, FileType type);
void CalculateFMean(ifstream& FilePath,ASCData& data,int pos,AcceCalibrationModule& cali);
void AcceCalibration(AcceCalibrationModule &cali, ofstream &resfile);
void CompensateAcc(ifstream &FilePath, ASCData &data, AcceCalibrationModule &cali, ofstream &resfile);
void CalculateRotAnglePair(ifstream& positive, ifstream& negative, int axis,
    GyroCalibrationModule& gyro, ofstream& positiveOutput, ofstream& negativeOutput);
void GyroCalibration(int Axis, GyroCalibrationModule &cali, int mode);
void CalculateGyroBias(ifstream &FilePath, ASCData &data, GyroCalibrationModule &cali, int mode);
void CompensateGyro(ifstream &FilePath, ASCData &data, int mode, GyroCalibrationModule &cali, ofstream &gyroresFile);

double GetDiffTime(GPSTIME &t1, GPSTIME &t2);
void CalculateCbn(AlignModule &align);
void EpochAlign(ifstream &FilePath, ASCData data, AlignModule &now, AlignModule &total, ofstream &alignfile);
void SecondAlign(ifstream &FilePath, ASCData data, AlignModule &now, AlignModule &total, ofstream &alignfile);
void AllAlign(ifstream &FilePath, ASCData &data, AlignModule &align, ofstream &alignres);
void CalTimeNoise(ifstream &FilePath, ASCData &data, ofstream &alignres);
