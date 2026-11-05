#include "IMU.h"

// 读取配置文件
bool LoadConfig(const string& filename, AppConfig& config)
{
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "Cannot open config file: " << filename << endl;
        return false;
    }

    // 重新初始化，避免重复调用时保留上一次配置
    config = AppConfig();

    string line;
    int lineNumber = 0;

    while (getline(file, line)) {
        lineNumber++;

        // 去掉可能存在的行尾回车符
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        // 跳过空行和注释
        if (line.empty()) {
            continue;
        }

        if (line[0] == '#') {
            continue;
        }

        // 每行必须包含等号
        if (line.find('=') == string::npos) {
            cerr << "Missing '=' at config line "
                 << lineNumber << endl;
            return false;
        }

        stringstream ss(line);
        string key;
        string value;

        getline(ss, key, '=');
        getline(ss, value);

        if (key.empty() || value.empty()) {
            cerr << "Empty key or value at config line "
                 << lineNumber << endl;
            return false;
        }

        // 功能编号
        if (key == "MODE") {
            if (value == "0") {
                config.mode = 0;
            }
            else if (value == "1") {
                config.mode = 1;
            }
            else if (value == "2") {
                config.mode = 2;
            }
            else {
                cerr << "MODE must be 0, 1, or 2." << endl;
                return false;
            }
        }

        // 单个文件路径
        else if (key == "ACC_RESULT_FILE") {
            config.accResultFile = value;
        }
        else if (key == "GYRO_RESULT_FILE") {
            config.gyroResultFile = value;
        }
        else if (key == "ALIGNMENT_INPUT_FILE") {
            config.alignmentInputFile = value;
        }
        else if (key == "ALIGNMENT_RESULT_WHOLE") {
            config.alignmentWholeResultFile = value;
        }
        else if (key == "ALIGNMENT_RESULT_SECOND") {
            config.alignmentSecondResultFile = value;
        }
        else if (key == "ALIGNMENT_RESULT_EPOCH") {
            config.alignmentEpochResultFile = value;
        }
        else if (key == "ALIGNMENT_RESULT_NOISE") {
            config.alignmentNoiseResultFile = value;
        }

        // 带编号的六个文件路径
        else {
            bool found = false;

            for (int i = 0; i < 6; i++) {
                string number = to_string(i + 1);

                if (key == "ACC_CALIBRATION_FILE_" + number) {
                    config.accCalibrationFiles[i] = value;
                    found = true;
                }
                else if (key == "ACC_COMPENSATION_FILE_" + number) {
                    config.accCompensationFiles[i] = value;
                    found = true;
                }
                else if (key == "ACC_COMPENSATION_RESULT_" + number) {
                    config.accCompensationResultFiles[i] = value;
                    found = true;
                }
                else if (key == "GYRO_ROTATION_FILE_" + number) {
                    config.gyroRotationFiles[i] = value;
                    found = true;
                }
                else if (key == "GYRO_COMPENSATION_FILE_" + number) {
                    config.gyroCompensationFiles[i] = value;
                    found = true;
                }
                else if (key == "GYRO_STATIC_FILE_" + number) {
                    config.gyroStaticFiles[i] = value;
                    found = true;
                }
                else if (key == "GYRO_ANGLE_RESULT_" + number) {
                    config.gyroAngleResultFiles[i] = value;
                    found = true;
                }
                else if (key == "GYRO_COMPENSATION_RESULT_" + number) {
                    config.gyroCompensationResultFiles[i] = value;
                    found = true;
                }

                if (found) {
                    break;
                }
            }

            if (!found) {
                cerr << "Unknown config key at line "
                     << lineNumber << ": " << key << endl;
                return false;
            }
        }
    }

    if (file.bad()) {
        cerr << "Failed to read config file." << endl;
        return false;
    }

    if (config.mode == -1) {
        cerr << "Missing MODE in config file." << endl;
        return false;
    }

    // 检查所选功能的配置是否完整
    if (config.mode == 0) {
        if (config.accResultFile.empty()) {
            cerr << "Missing ACC_RESULT_FILE." << endl;
            return false;
        }

        for (int i = 0; i < 6; i++) {
            if (config.accCalibrationFiles[i].empty() ||
                config.accCompensationFiles[i].empty() ||
                config.accCompensationResultFiles[i].empty()) {
                cerr << "Incomplete accelerometer config for position "
                     << i + 1 << endl;
                return false;
            }
        }
    }
    else if (config.mode == 1) {
        if (config.gyroResultFile.empty()) {
            cerr << "Missing GYRO_RESULT_FILE." << endl;
            return false;
        }

        for (int i = 0; i < 6; i++) {
            if (config.gyroRotationFiles[i].empty() ||
                config.gyroCompensationFiles[i].empty() ||
                config.gyroStaticFiles[i].empty() ||
                config.gyroAngleResultFiles[i].empty() ||
                config.gyroCompensationResultFiles[i].empty()) {
                cerr << "Incomplete gyroscope config for file "
                     << i + 1 << endl;
                return false;
            }
        }
    }
    else if (config.mode == 2) {
        if (config.alignmentInputFile.empty() ||
            config.alignmentWholeResultFile.empty() ||
            config.alignmentSecondResultFile.empty() ||
            config.alignmentEpochResultFile.empty() ||
            config.alignmentNoiseResultFile.empty()) {
            cerr << "Incomplete alignment config." << endl;
            return false;
        }
    }

    return true;
}