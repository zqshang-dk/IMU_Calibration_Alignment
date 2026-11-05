#include "IMU.h"

// int main()
// {
// 	int mode = -1;

// 	cout << "0 - Accelerometer calibration\n"
// 		<< "1 - Gyroscope calibration\n"
// 		<< "2 - Initial alignment\n"
// 		<< "Enter the function number (0, 1, or 2): ";

// 	// 检查是否成功读入整数
// 	if (!(cin >> mode)) {
// 		cerr << "\nFailed to read a function number. "
// 			<< "Please enter 0, 1, or 2.\n";
// 		return 1;
// 	}
// 	// 检查编号是否在有效范围内
// 	if (mode < 0 || mode > 2) {
// 		cerr << "\nInvalid function number: " << mode
// 			<< ". Please restart the program and enter 0, 1, or 2.\n";
// 		return 1;
// 	}

// 	switch (mode){

// 	/* 加速度计标定 */
// 	case 0: {
// 		// 读取文件路径
// 		string filename_1 = "work\\asc\\calibration\\X_up.ASC";
// 		string filename_2 = "work\\asc\\calibration\\X_down.ASC";
// 		string filename_3 = "work\\asc\\calibration\\Y_up.ASC";
// 		string filename_4 = "work\\asc\\calibration\\Y_down.ASC";
// 		string filename_5 = "work\\asc\\calibration\\Z_up.ASC";
// 		string filename_6 = "work\\asc\\calibration\\Z_down.ASC";
// 		string filename_c1 = "work\\asc\\calibration\\X_up.ASC";
// 		string filename_c2 = "work\\asc\\calibration\\X_down.ASC";
// 		string filename_c3 = "work\\asc\\calibration\\Y_up.ASC";
// 		string filename_c4 = "work\\asc\\calibration\\Y_down.ASC";
// 		string filename_c5 = "work\\asc\\calibration\\Z_up.ASC";
// 		string filename_c6 = "work\\asc\\calibration\\Z_down.ASC";

// 		string filesave = "work\\results\\calibration\\AccCaliError.txt";
// 		string filesave_c1 = "work\\results\\calibration\\X_up_Raw_Com.txt";
// 		string filesave_c2 = "work\\results\\calibration\\X_down_Raw_Com.txt";
// 		string filesave_c3 = "work\\results\\calibration\\Y_up_Raw_Com.txt";
// 		string filesave_c4 = "work\\results\\calibration\\Y_down_Raw_Com.txt";
// 		string filesave_c5 = "work\\results\\calibration\\Z_up_Raw_Com.txt";
// 		string filesave_c6 = "work\\results\\calibration\\Z_down_Raw_Com.txt";

// 		ASCData data;
// 		AcceCalibrationModule accecali;

// 		// 打开文件进行读取和写入
// 		ifstream file_1(filename_1), file_2(filename_2), file_3(filename_3), file_4(filename_4), file_5(filename_5), file_6(filename_6);
// 		ifstream file_c1(filename_c1), file_c2(filename_c2), file_c3(filename_c3), file_c4(filename_c4), file_c5(filename_c5), file_c6(filename_c6);
// 		ofstream file(filesave), file_s1(filesave_c1), file_s2(filesave_c2), file_s3(filesave_c3), file_s4(filesave_c4), file_s5(filesave_c5), file_s6(filesave_c6);
// 		if (!(file_1 || file_2 || file_3 || file_4 || file_5 || file_6 || file)) { cerr << "\nCannot open the file to read rawdata!\n"; return -1; }
// 		if (!(file_c1 || file_c2 || file_c3 || file_c4 || file_c5 || file_c6 || file_s1 || file_s2 || file_s3 || file_s4 || file_s5 || file_s6)) { cerr << "\nCannot open the file to read rawdata!\n"; return -1; }

// 		// 逐个位置求加速度计标定数据
// 		CalculateFMean(file_1, data, 0, accecali);	// 第一个位置
// 		CalculateFMean(file_2, data, 1, accecali);
// 		CalculateFMean(file_3, data, 2, accecali);
// 		CalculateFMean(file_4, data, 3, accecali);
// 		CalculateFMean(file_5, data, 4, accecali);
// 		CalculateFMean(file_6, data, 5, accecali);

// 		AcceCalibration(accecali, file);	// 加速度计标定

// 		// 数据补偿
// 		CompensateAcc(file_c1, data, accecali, file_s1);
// 		CompensateAcc(file_c2, data, accecali, file_s2);
// 		CompensateAcc(file_c3, data, accecali, file_s3);
// 		CompensateAcc(file_c4, data, accecali, file_s4);
// 		CompensateAcc(file_c5, data, accecali, file_s5);
// 		CompensateAcc(file_c6, data, accecali, file_s6);

// 		file_1.close(); file_2.close(); file_3.close(); file_4.close(); file_5.close(); file_6.close();	// 关闭文件
// 		file.close();
// 		file_c1.close(); file_c2.close(); file_c3.close(); file_c4.close(); file_c5.close(); file_c6.close();
// 		file_s1.close(); file_s2.close(); file_s3.close(); file_s4.close(); file_s5.close(); file_s6.close();

// 		break;
// 	}

// 	/* 陀螺仪标定 */
// 	case 1: {
// 		// 读取文件路径
// 		string filename_1 = "work\\asc\\calibration\\X_down_360.ASC";
// 		string filename_2 = "work\\asc\\calibration\\X_down_-360.ASC";
// 		string filename_3 = "work\\asc\\calibration\\Y_down_360.ASC";
// 		string filename_4 = "work\\asc\\calibration\\Y_down_-360.ASC";
// 		string filename_5 = "work\\asc\\calibration\\Z_down_360.ASC";
// 		string filename_6 = "work\\asc\\calibration\\Z_down_-360.ASC";
// 		string filename_c1 = "work\\asc\\calibration\\X_down_360.ASC";
// 		string filename_c2 = "work\\asc\\calibration\\X_down_-360.ASC";
// 		string filename_c3 = "work\\asc\\calibration\\Y_down_360.ASC";
// 		string filename_c4 = "work\\asc\\calibration\\Y_down_-360.ASC";
// 		string filename_c5 = "work\\asc\\calibration\\Z_down_360.ASC";
// 		string filename_c6 = "work\\asc\\calibration\\Z_down_-360.ASC";
// 		string filename_acc1 = "work\\asc\\calibration\\X_up.ASC";
// 		string filename_acc2 = "work\\asc\\calibration\\X_down.ASC";
// 		string filename_acc3 = "work\\asc\\calibration\\Y_up.ASC";
// 		string filename_acc4 = "work\\asc\\calibration\\Y_down.ASC";
// 		string filename_acc5 = "work\\asc\\calibration\\Z_up.ASC";
// 		string filename_acc6 = "work\\asc\\calibration\\Z_down.ASC";

// 		string filesave = "work\\results\\alignment\\GyroCalibration.txt";
// 		string filesave1 = "work\\results\\alignment\\x+360_Angle.txt";
// 		string filesave2 = "work\\results\\alignment\\x-360_Angle.txt";
// 		string filesave3 = "work\\results\\alignment\\y+360_Angle.txt";
// 		string filesave4 = "work\\results\\alignment\\y-360_Angle.txt";
// 		string filesave5 = "work\\results\\alignment\\z+360_Angle.txt";
// 		string filesave6 = "work\\results\\alignment\\z-360_Angle.txt";
// 		string filesave_s1 = "work\\results\\alignment\\x+360_Raw_Com.txt";
// 		string filesave_s2 = "work\\results\\alignment\\x-360_Raw_Com.txt";
// 		string filesave_s3 = "work\\results\\alignment\\y+360_Raw_Com.txt";
// 		string filesave_s4 = "work\\results\\alignment\\y-360_Raw_Com.txt";
// 		string filesave_s5 = "work\\results\\alignment\\z+360_Raw_Com.txt";
// 		string filesave_s6 = "work\\results\\alignment\\z-360_Raw_Com.txt";

// 		ASCData data;
// 		GyroCalibrationModule error_x, error_y, error_z;

// 		// 打开文件进行读取和写入
// 		ifstream file_1(filename_1), file_2(filename_2), file_3(filename_3), file_4(filename_4), file_5(filename_5), file_6(filename_6);
// 		ifstream file_c1(filename_c1), file_c2(filename_c2), file_c3(filename_c3), file_c4(filename_c4), file_c5(filename_c5), file_c6(filename_c6);
// 		ifstream file_acc1(filename_acc1), file_acc2(filename_acc2), file_acc3(filename_acc3), file_acc4(filename_acc4), file_acc5(filename_acc5), file_acc6(filename_acc6);
// 		ofstream file(filesave), file1(filesave1), file2(filesave2), file3(filesave3), file4(filesave4), file5(filesave5), file6(filesave6);
// 		ofstream file_s1(filesave_s1), file_s2(filesave_s2), file_s3(filesave_s3), file_s4(filesave_s4), file_s5(filesave_s5), file_s6(filesave_s6);
// 		if (!(file_1 || file_2 || file_3 || file_4 || file_5 || file_6 || file)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
// 		if (!(file_c1 || file_c2 || file_c3 || file_c4 || file_c5 || file_c6)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
// 		if (!(file_s1 || file_s2 || file_s3 || file_s4 || file_s5 || file_s6)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
// 		if (!(file_acc1 || file_acc2 || file_acc3 || file_acc4 || file_acc5 || file_acc6)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }

// 		// 计算正转和反转角度累加
// 		CalculateRotAngle(file_1, data, 0, 0, error_x, file1);
// 		CalculateRotAngle(file_2, data, 1, 0, error_x, file2);
// 		CalculateRotAngle(file_3, data, 0, 1, error_y, file3);
// 		CalculateRotAngle(file_4, data, 1, 1, error_y, file4);
// 		CalculateRotAngle(file_5, data, 0, 2, error_z, file5);
// 		CalculateRotAngle(file_6, data, 1, 2, error_z, file6);

// 		GyroCalibration(0, error_x, 0);	// 计算陀螺仪标度因数
// 		GyroCalibration(1, error_y, 0);
// 		GyroCalibration(2, error_z, 0);

// 		CalculateGyroBias(file_acc1, data, error_x, 0);
// 		CalculateGyroBias(file_acc2, data, error_x, 1);
// 		CalculateGyroBias(file_acc3, data, error_y, 0);
// 		CalculateGyroBias(file_acc4, data, error_y, 1);
// 		CalculateGyroBias(file_acc5, data, error_z, 0);
// 		CalculateGyroBias(file_acc6, data, error_z, 1);

// 		GyroCalibration(0, error_x, 1);	// 计算陀螺仪零偏
// 		GyroCalibration(1, error_y, 1);
// 		GyroCalibration(2, error_z, 1);
// 		cout << "\nGyroscope calibration completed. "<< "Result files have been generated.\n";

// 		// 整合三轴标定结果
// 		GyroCalibrationModule error;	// 将三轴结果合并为一个完整的误差对象
// 		error.bias[0] = error_x.bias[0]; error.bias[1] = error_y.bias[1]; error.bias[2] = error_z.bias[2];
// 		error.scale[0] = error_x.scale[0]; error.scale[1] = error_y.scale[1]; error.scale[2] = error_z.scale[2];

// 		CompensateGyro(file_c1, data, 0, error, file_s1);
// 		CompensateGyro(file_c2, data, 0, error, file_s2);
// 		CompensateGyro(file_c3, data, 1, error, file_s3);
// 		CompensateGyro(file_c4, data, 1, error, file_s4);
// 		CompensateGyro(file_c5, data, 2, error, file_s5);
// 		CompensateGyro(file_c6, data, 2, error, file_s6);

// 		// 保存标定结果
// 		file << fixed << setprecision(12);
// 		file << "%Bias_XYZ(deg/h):\n"
// 			<< error_x.bias[0] << "," << error_y.bias[1] << "," << error_z.bias[2] << endl;
// 		file << "%Scale(XYZ):\n"
// 			<< error_x.scale[0] << "," << error_y.scale[1] << "," << error_z.scale[2];

// 		// 关闭文件
// 		file_1.close(); file_2.close(); file_3.close(); file_4.close(); file_5.close(); file_6.close();
// 		file.close(); file1.close(); file2.close(); file3.close(); file4.close(); file5.close(); file6.close();
// 		break;
// 	}

// 	/* 初始对准 */
// 	case 2:
// 	{
// 		// 对准数据文件
// 		string filename1 = "Data\\Align\\Align_30min.ASC";
// 		string filename2 = "Data\\Align\\Align_30min.ASC";
// 		string filename3 = "Data\\Align\\Align_30min.ASC";
// 		string filename4 = "Data\\Align\\Align_30min.ASC";
		
// 		// 对准结果保存文件
// 		string filename_save1 = "Result\\Figure_Matlab\\Align\\Align_Whole.txt";
// 		string filename_save2 = "Result\\Figure_Matlab\\Align\\Align_Second.txt";
// 		string filename_save3 = "Result\\Figure_Matlab\\Align\\Align_Epoch.txt";
// 		string filename_save4 = "Result\\Figure_Matlab\\Align\\Align_Deviation.txt";

// 		// 对准变量定义
// 		ASCData rawdata;
// 		AlignModule pos1, pos2, pos3;

// 		// 打开文件进行读取和写入
// 		ifstream file1(filename1), file2(filename2), file3(filename3), file4(filename4);
// 		ofstream file_save1(filename_save1), file_save2(filename_save2), file_save3(filename_save3), file_save4(filename_save4);
// 		if (!file1 || !file_save1 || !file_save2 || !file_save3 || !file_save4) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }

// 		// 执行三种对准方式
// 		AllAlign(file1, rawdata, pos1, file_save1);	// 整体粗对准，作为参考基准
// 		SecondAlign(file2, rawdata, pos2, pos1, file_save2);	// 每秒累积数据进行粗对准，与参考基准对比
// 		EpochAlign(file3, rawdata, pos3, pos1, file_save3);	// 每个历元数据进行粗对准，与参考基准对比

// 		CalTimeNoise(file4, rawdata, file_save4);	// 计算时间噪声特性

// 		file1.close(); file2.close(); file3.close(); file4.close();	// 关闭文件
// 		file_save1.close();
// 		file_save2.close();
// 		file_save3.close();
// 		file_save4.close();

// 		break;
// 	}
// 	}
// 	return 0;
// }


int main(int argc, char* argv[])
{
    try {
    AppConfig config;

    // 默认读取原配置；允许指定独立验证配置，不必修改原文件。
    if (argc > 2) {
        cerr << "Usage: IMU.exe [config.txt]" << endl;
        return 1;
    }
    if (!LoadConfig(argc == 2 ? argv[1] : "src/config.txt", config)) {
        return 1;
    }

    cout << "Selected mode: " << config.mode << endl;

    switch (config.mode) {

    // ========================================================
    // 加速度计标定
    // ========================================================
    case 0: {
        ifstream file[6];
        ifstream file_c[6];

        ofstream result;
        ofstream result_c[6];

        // 先打开并检查全部输入文件
        for (int i = 0; i < 6; i++) {
            file[i].open(config.accCalibrationFiles[i]);

            if (!file[i]) {
                cerr << "Cannot open input file: "
                     << config.accCalibrationFiles[i] << endl;
                return 1;
            }

            file_c[i].open(config.accCompensationFiles[i]);

            if (!file_c[i]) {
                cerr << "Cannot open input file: "
                     << config.accCompensationFiles[i] << endl;
                return 1;
            }
        }

        // 打开标定参数输出文件
        result.open(config.accResultFile);

        if (!result) {
            cerr << "Cannot open output file: "
                 << config.accResultFile << endl;
            return 1;
        }

        // 打开补偿结果文件
        for (int i = 0; i < 6; i++) {
            result_c[i].open(config.accCompensationResultFiles[i]);

            if (!result_c[i]) {
                cerr << "Cannot open output file: "
                     << config.accCompensationResultFiles[i] << endl;
                return 1;
            }
        }

        ASCData data;
        data.ft = CALIBRATION;

        AcceCalibrationModule cali;

        // 六位置均值
        for (int i = 0; i < 6; i++) {
            CalculateFMean(file[i], data, i, cali);
        }

        // 求解加速度计标定参数
        AcceCalibration(cali, result);

        // 对六个文件分别补偿
        for (int i = 0; i < 6; i++) {
            CompensateAcc(file_c[i], data, cali, result_c[i]);
        }

        // 关闭输出并检查写入状态
        result.close();

        if (!result) {
            cerr << "Failed to save: "
                 << config.accResultFile << endl;
            return 1;
        }

        for (int i = 0; i < 6; i++) {
            result_c[i].close();

            if (!result_c[i]) {
                cerr << "Failed to save: "
                     << config.accCompensationResultFiles[i] << endl;
                return 1;
            }
        }

        cout << "Accelerometer processing completed." << endl;
        break;
    }

    // ========================================================
    // 陀螺仪标定
    // ========================================================
    case 1: {
        ifstream file[6];
        ifstream file_c[6];
        ifstream file_acc[6];

        ofstream result;
        ofstream result_angle[6];
        ofstream result_c[6];

        // 打开旋转、补偿和静态数据
        for (int i = 0; i < 6; i++) {
            file[i].open(config.gyroRotationFiles[i]);

            if (!file[i]) {
                cerr << "Cannot open input file: "
                     << config.gyroRotationFiles[i] << endl;
                return 1;
            }

            file_c[i].open(config.gyroCompensationFiles[i]);

            if (!file_c[i]) {
                cerr << "Cannot open input file: "
                     << config.gyroCompensationFiles[i] << endl;
                return 1;
            }

            file_acc[i].open(config.gyroStaticFiles[i]);

            if (!file_acc[i]) {
                cerr << "Cannot open input file: "
                     << config.gyroStaticFiles[i] << endl;
                return 1;
            }
        }

        // 打开标定参数输出文件
        result.open(config.gyroResultFile);

        if (!result) {
            cerr << "Cannot open output file: "
                 << config.gyroResultFile << endl;
            return 1;
        }

        // 打开积分角与补偿结果文件
        for (int i = 0; i < 6; i++) {
            result_angle[i].open(config.gyroAngleResultFiles[i]);

            if (!result_angle[i]) {
                cerr << "Cannot open output file: "
                     << config.gyroAngleResultFiles[i] << endl;
                return 1;
            }

            result_c[i].open(config.gyroCompensationResultFiles[i]);

            if (!result_c[i]) {
                cerr << "Cannot open output file: "
                     << config.gyroCompensationResultFiles[i] << endl;
                return 1;
            }
        }

        ASCData data;
        data.ft = CALIBRATION;

        // 分别保存 X、Y、Z 轴的标定结果
        GyroCalibrationModule error[3];

        for (int axis = 0; axis < 3; axis++) {
            // 正转文件：0、2、4
            // 反转文件：1、3、5
            int positive = 2 * axis;
            int negative = 2 * axis + 1;

            // 正反转角度积分
            CalculateRotAnglePair(file[positive], file[negative], axis,
                error[axis], result_angle[positive], result_angle[negative]);

            // 根据正反转结果计算比例因子
            GyroCalibration(axis, error[axis], 0);

            // 对应轴朝上、朝下的静态均值
            CalculateGyroBias(
                file_acc[positive], data, error[axis], 0);

            CalculateGyroBias(
                file_acc[negative], data, error[axis], 1);

            // 根据静态结果计算零偏
            GyroCalibration(axis, error[axis], 1);
        }

        // 合并三轴参数
        GyroCalibrationModule cali;

        for (int axis = 0; axis < 3; axis++) {
            cali.bias[axis] = error[axis].bias[axis];
            cali.scale[axis] = error[axis].scale[axis];
        }

        // 对六个旋转文件分别补偿
        for (int i = 0; i < 6; i++) {
            int axis = i / 2;

            CompensateGyro(
                file_c[i], data, axis, cali, result_c[i]);
        }

        // 保存参数
        result << fixed << setprecision(12);

        result << "%Bias_XYZ(deg/h):\n"
               << cali.bias[0] * rad2deg * 3600.0 << ","
               << cali.bias[1] * rad2deg * 3600.0 << ","
               << cali.bias[2] * rad2deg * 3600.0 << "\n";

        result << "%Scale(XYZ):\n"
               << cali.scale[0] << ","
               << cali.scale[1] << ","
               << cali.scale[2] << "\n";

        result.close();

        if (!result) {
            cerr << "Failed to save: "
                 << config.gyroResultFile << endl;
            return 1;
        }

        for (int i = 0; i < 6; i++) {
            result_angle[i].close();
            result_c[i].close();

            if (!result_angle[i] || !result_c[i]) {
                cerr << "Failed to save gyroscope results for file "
                     << i + 1 << endl;
                return 1;
            }
        }

        cout << "Gyroscope processing completed." << endl;
        break;
    }

    // ========================================================
    // 初始对准
    // ========================================================
    case 2: {
        // 四个函数分别从文件开头读取
        ifstream file1(config.alignmentInputFile);
        ifstream file2(config.alignmentInputFile);
        ifstream file3(config.alignmentInputFile);
        ifstream file4(config.alignmentInputFile);

        if (!file1 || !file2 || !file3 || !file4) {
            cerr << "Cannot open alignment input file: "
                 << config.alignmentInputFile << endl;
            return 1;
        }

        ofstream result1(config.alignmentWholeResultFile);
        ofstream result2(config.alignmentSecondResultFile);
        ofstream result3(config.alignmentEpochResultFile);
        ofstream result4(config.alignmentNoiseResultFile);

        if (!result1 || !result2 || !result3 || !result4) {
            cerr << "Cannot open alignment output files." << endl;
            return 1;
        }

        ASCData data;
        data.ft = ALIGNMENT;

        AlignModule pos1;
        AlignModule pos2;
        AlignModule pos3;

        AllAlign(file1, data, pos1, result1);
        SecondAlign(file2, data, pos2, pos1, result2);
        EpochAlign(file3, data, pos3, pos1, result3);
        CalTimeNoise(file4, data, result4);

        result1.close();
        result2.close();
        result3.close();
        result4.close();

        if (!result1 || !result2 || !result3 || !result4) {
            cerr << "Failed to save alignment results." << endl;
            return 1;
        }

        cout << "Initial alignment processing completed." << endl;
        break;
    }

    default:
        cerr << "Invalid MODE." << endl;
        return 1;
    }

    return 0;
    } catch (const exception& error) {
        cerr << "IMU processing failed: " << error.what() << endl;
        return 1;
    }
}
