#include "IMU.h"

namespace {
double Number(const string& value) {
    size_t used = 0;
    const double x = stod(value, &used);
    if (used != value.size() || !isfinite(x))
        throw runtime_error("Invalid numeric IMU field: " + value);
    return x;
}

uint32_t Crc32(const string& body) {
    uint32_t crc = 0;
    for (unsigned char c : body) {
        crc ^= c;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0u);
    }
    return crc;
}
}

bool ReadFile(ifstream& file, ASCData& data) {
    const bool alignment = data.ft == ALIGNMENT;
    const string prefix = alignment ? "#RAWIMUXA," : "%RAWIMUSA,";
    const double frequency = alignment ? FNSC : FXWG;
    const double qa = alignment ? AS_NSC : AS_XWG;
    const double qg = alignment ? GS_NSC : GS_XWG;
    string line;
    while (getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.compare(0, prefix.size(), prefix) != 0) continue;
        const auto semi = line.find(';'), star = line.find('*');
        if (semi == string::npos || star == string::npos || star <= semi)
            throw runtime_error("Malformed IMU record");
        size_t used = 0;
        const string check = line.substr(star + 1);
        const auto expected = stoul(check, &used, 16);
        if (used != check.size() || expected != Crc32(line.substr(1, star - 1)))
            throw runtime_error("IMU CRC mismatch");
        vector<string> fields;
        string field;
        stringstream ss(line.substr(semi + 1, star - semi - 1));
        while (getline(ss, field, ',')) fields.push_back(field);
        if (fields.size() != (alignment ? 11u : 9u))
            throw runtime_error("Incorrect IMU field count");
        const size_t j = alignment ? 2 : 0;
        ASCData next;
        next.ft = data.ft;
        const double week = Number(fields[j]);
        if (week < 0 || week > 100000 || week != floor(week))
            throw runtime_error("Invalid GPS week");
        next.time.Week = static_cast<int>(week);
        next.time.SOW = Number(fields[j + 1]); // 完整浮点字符串，兼容6位/9位小数
        if (next.time.SOW < 0 || next.time.SOW >= 604800)
            throw runtime_error("Invalid GPS seconds of week");
        const int order[3] = {5, 4, 3};
        const double sign[3] = {1, -1, 1};
        for (int axis = 0; axis < 3; ++axis) {
            next.acce.acce[axis] = sign[axis] * Number(fields[j + order[axis]]) * qa * frequency;
            next.gyro.gyro[axis] = sign[axis] * Number(fields[j + order[axis] + 3]) * qg * frequency;
        }
        data = next;
        return true;
    }
    if (file.bad()) throw runtime_error("Failed reading IMU input");
    return false;
}

vector<ASCData> ReadRecords(ifstream& file, FileType type) {
    vector<ASCData> rows;
    ASCData data;
    data.ft = type;
    while (ReadFile(file, data)) {
        if (!rows.empty()) {
            const auto& prev = rows.back().time;
            const double dt = (data.time.Week - prev.Week) * 604800.0 + data.time.SOW - prev.SOW;
            if (dt <= 0) throw runtime_error("Non-increasing IMU timestamps");
        }
        rows.push_back(data);
    }
    if (rows.empty()) throw runtime_error("No valid IMU records");
    return rows;
}
