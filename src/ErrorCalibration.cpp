#include "IMU.h"

namespace {
double Elapsed(const GPSTIME& first, const GPSTIME& now) {
    return (now.Week - first.Week) * 604800.0 + now.SOW - first.SOW;
}

Vector3d StaticMean(ifstream& file, bool gyroscope) {
    const auto rows = ReadRecords(file, CALIBRATION);
    Vector3d sum = Vector3d::Zero();
    size_t count = 0;
    const double end = Elapsed(rows.front().time, rows.back().time) - StaticTrimSeconds;
    for (const auto& row : rows) {
        const double t = Elapsed(rows.front().time, row.time);
        if (t < StaticTrimSeconds || t > end) continue;
        for (int j = 0; j < 3; ++j)
            sum[j] += gyroscope ? row.gyro.gyro[j] : row.acce.acce[j];
        ++count;
    }
    if (!count) throw runtime_error("Static record too short after trimming");
    return sum / static_cast<double>(count);
}

pair<int, int> ActiveRange(const vector<ASCData>& rows, int axis) {
    if (rows.size() < 400) throw runtime_error("Rotation record too short");
    double baseline = 0;
    for (int i = 0; i < 200; ++i) baseline += rows[i].gyro.gyro[axis] * rad2deg / 200.0;
    int first = -1, last = -1;
    for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
        double smoothed = 0;
        for (int k = max(0, i - 25); k <= min(static_cast<int>(rows.size()) - 1, i + 25); ++k)
            smoothed += (rows[k].gyro.gyro[axis] * rad2deg - baseline) / 51.0;
        if (abs(smoothed) > 0.1) {
            if (first < 0) first = i;
            last = i;
        }
    }
    if (first < 0) throw runtime_error("No complete rotation found");
    return {first, last};
}
}

void CalculateFMean(ifstream& file, ASCData& data, int pos, AcceCalibrationModule& cali) {
    data.ft = CALIBRATION;
    cali.fmean.col(pos) = StaticMean(file, false);
}

void AcceCalibration(AcceCalibrationModule& cali, ofstream& out) {
    MatrixXd truth(4, 6);
    truth << gp, -gp, 0, 0, 0, 0,
             0, 0, gp, -gp, 0, 0,
             0, 0, 0, 0, gp, -gp,
             1, 1, 1, 1, 1, 1;
    cali.Caliresult = truth.transpose().colPivHouseholderQr().solve(cali.fmean.transpose()).transpose();
    cali.AcceBias();
    cali.AcceScale();
    cali.AcceCross();
    out << fixed << setprecision(12);
    out << "%Bias(m/s2)\t" << cali.bias[0] << ',' << cali.bias[1] << ',' << cali.bias[2] << '\n';
    out << "%Scale\t" << cali.scale[0] << ',' << cali.scale[1] << ',' << cali.scale[2] << '\n';
    out << "%Cross(output_axis,input_axis)\n" << cali.cross << '\n';
}

void CompensateAcc(ifstream& file, ASCData& data, AcceCalibrationModule& cali, ofstream& out) {
    data.ft = CALIBRATION;
    const Matrix3d response = cali.Caliresult.block<3, 3>(0, 0);
    const auto solver = response.fullPivLu();
    if (!solver.isInvertible()) throw runtime_error("Singular accelerometer response");
    const Vector3d bias = cali.Caliresult.col(3);
    out << "%Time(s),Comp_X,Comp_Y,Comp_Z,Raw_X,Raw_Y,Raw_Z(m/s2)\n";
    while (ReadFile(file, data)) {
        const Vector3d raw(data.acce.rax, data.acce.ray, data.acce.raz);
        const Vector3d comp = solver.solve(raw - bias);
        out << fixed << setprecision(9) << data.time.SOW << ',' << setprecision(12)
            << comp[0] << ',' << comp[1] << ',' << comp[2] << ','
            << raw[0] << ',' << raw[1] << ',' << raw[2] << '\n';
    }
}

// 同时确定正反转积分窗：完整动作 + 静态留边，轴内样本数严格相等。
void CalculateRotAnglePair(ifstream& positive, ifstream& negative, int axis,
    GyroCalibrationModule& gyro, ofstream& positiveOutput, ofstream& negativeOutput) {
    if (axis < 0 || axis > 2) throw runtime_error("Invalid gyro axis");
    const vector<ASCData> rows[2] = {ReadRecords(positive, CALIBRATION), ReadRecords(negative, CALIBRATION)};
    const pair<int, int> active[2] = {ActiveRange(rows[0], axis), ActiveRange(rows[1], axis)};
    const int length = max(active[0].second - active[0].first + 1,
                           active[1].second - active[1].first + 1) + 400;
    ofstream* outputs[2] = {&positiveOutput, &negativeOutput};
    for (int direction = 0; direction < 2; ++direction) {
        const auto& r = rows[direction];
        const int start = max(0, min(static_cast<int>(r.size()) - length,
            static_cast<int>(floor((active[direction].first + active[direction].second - length) / 2.0))));
        const int stop = start + length;
        if (stop > static_cast<int>(r.size()) || start > active[direction].first - 100 ||
            stop < active[direction].second + 100)
            throw runtime_error("Insufficient stationary margin around rotation");
        Vector3d angle = Vector3d::Zero();
        auto& out = *outputs[direction];
        out << "%Time(s),DeltaTheta_X,DeltaTheta_Y,DeltaTheta_Z(deg)\n";
        out << "%Window_n=" << length << ",T_s=" << fixed << setprecision(9) << length / FXWG
            << ",start_sow=" << r[start].time.SOW << ",end_sow=" << r[stop - 1].time.SOW << '\n';
        for (int k = start; k < stop; ++k) {
            if (k > start && abs(Elapsed(r[k - 1].time, r[k].time) - 1.0 / FXWG) > 1e-6)
                throw runtime_error("Missing samples inside rotation window");
            out << setprecision(9) << r[k].time.SOW;
            for (int j = 0; j < 3; ++j) {
                // 输入为区间平均角速度，直接恢复并累加增量，不再做梯形积分。
                const double delta = r[k].gyro.gyro[j] / FXWG;
                angle[j] += delta;
                out << ',' << setprecision(12) << delta * rad2deg;
            }
            out << '\n';
        }
        if ((direction == 0 && angle[axis] <= 0) || (direction == 1 && angle[axis] >= 0))
            throw runtime_error("Rotation direction disagrees with positive/negative file order");
        auto& stored = direction == 0 ? gyro.posangle : gyro.negangle;
        for (int j = 0; j < 3; ++j) stored.gyro[j] = angle[j]; // rad
        (direction == 0 ? gyro.posT : gyro.negT) = length / FXWG;
        out << "%IntegratedAngle(deg)=" << angle[axis] * rad2deg << '\n';
    }
}

void GyroCalibration(int axis, GyroCalibrationModule& cali, int mode) {
    if (axis < 0 || axis > 2) throw runtime_error("Invalid gyro axis");
    if (mode == 0) {
        if (cali.posT <= 0 || abs(cali.posT - cali.negT) > 1e-10)
            throw runtime_error("Gyro rotation windows must have equal duration");
        cali.scale[axis] = (cali.posangle.gyro[axis] - cali.negangle.gyro[axis])
            / (2.0 * Circle * deg2rad) - 1.0;
        const double earth = cali.verticalSign * OMEGA * sin(Latitude);
        cali.bias[axis] = (cali.posangle.gyro[axis] + cali.negangle.gyro[axis])
            / (2.0 * cali.posT) - (1.0 + cali.scale[axis]) * earth;
    } else if (mode == 1) {
        if (!cali.hasStaticUp || !cali.hasStaticDown)
            throw runtime_error("Both static orientations are required");
        cali.bias[axis] = (cali.staticUp.gyro[axis] + cali.staticDown.gyro[axis]) / 2.0;
    } else throw runtime_error("Invalid gyro calibration mode");
}

void CalculateGyroBias(ifstream& file, ASCData& data, GyroCalibrationModule& cali, int mode) {
    data.ft = CALIBRATION;
    if (mode != 0 && mode != 1) throw runtime_error("Invalid static orientation");
    const Vector3d mean = StaticMean(file, true);
    auto& target = mode == 0 ? cali.staticUp : cali.staticDown;
    for (int j = 0; j < 3; ++j) target.gyro[j] = mean[j];
    (mode == 0 ? cali.hasStaticUp : cali.hasStaticDown) = true;
}

void CompensateGyro(ifstream& file, ASCData& data, int axis, GyroCalibrationModule& cali, ofstream& out) {
    if (axis < 0 || axis > 2 || abs(1 + cali.scale[axis]) < 1e-12)
        throw runtime_error("Invalid gyro compensation parameters");
    data.ft = CALIBRATION;
    const double earth = cali.verticalSign * OMEGA * sin(Latitude);
    out << "%Time(s),TurntableRate,RawRate,CompensatedInertialRate(rad/s)\n";
    while (ReadFile(file, data)) {
        const double raw = data.gyro.gyro[axis];
        const double inertial = (raw - cali.bias[axis]) / (1 + cali.scale[axis]);
        // 地球自转只从转台角速度比较列扣除，惯性角速度列保留它。
        out << fixed << setprecision(9) << data.time.SOW << ',' << setprecision(12)
            << inertial - earth << ',' << raw << ',' << inertial << '\n';
    }
}
