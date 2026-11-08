#include "IMU.h"

using namespace std;

bool ReadFile(ifstream& file,ASCData& data){
    double f, converta, convertg;
    if (data.ft == CALIBRATION){
        f = FXWG;
        converta = AS_XWG;
        convertg = GS_XWG;
    }
    else if(data.ft==ALIGNMENT){
        f = FNSC;
        converta = AS_NSC;
        convertg = GS_NSC;
    }
        
    // 读取文件内容
    string line;
    if(getline(file,line)){
        // 如果为空就结束循环
        if(line.empty()){
            return false;
        }
        stringstream ss(line);
        string part;

        getline(ss, part, ';');
        if(data.ft==ALIGNMENT){
            getline(ss, part, ',');
		    getline(ss, part, ',');
        }

        //读取时间数据
        getline(ss, part, ',');
        data.time.Week = stoi(part);
        getline(ss, part, '.');
        data.time.SOW = stod(part);
        getline(ss, part, ',');
        data.time.SOW += stod(part) * 1e-9;

        getline(ss, part, ',');

        // 读取加速度计数据
        getline(ss, part, ',');
        data.acce.raz = stod(part) * converta * f;
        getline(ss, part, ',');
        data.acce.ray = -stod(part) * converta * f;
        getline(ss, part, ',');
        data.acce.rax = stod(part) * converta * f;

        getline(ss, part, ',');
        data.gyro.rgz = stod(part) * convertg * f;
        getline(ss, part, ',');
        data.gyro.rgy = -stod(part) * convertg * f;
        getline(ss, part, '*');
        data.gyro.rgx = stod(part) * convertg * f;

        return true;
    }
    return false;
}