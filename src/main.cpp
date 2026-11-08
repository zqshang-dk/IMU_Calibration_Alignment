#include "IMU.h"

int main()
{
    AppConfig config;

    // 从项目根目录运行时，读取 src/config
    if (!LoadConfig("src/config.txt", config)) {
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
            CalculateRotAngle(file[positive], data, 0, axis,error[axis], result_angle[positive]);

            CalculateRotAngle(file[negative], data, 1, axis,error[axis], result_angle[negative]);

            // 根据正反转结果计算比例因子
            // 对应轴朝上、朝下的静态均值
            CalculateGyroBias(file_acc[positive], data, error[axis], 0);
            CalculateGyroBias(file_acc[negative], data, error[axis], 1);

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
            CompensateGyro(file_c[i], data, axis, cali, result_c[i]);
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
}
