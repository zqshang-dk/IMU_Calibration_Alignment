#include "IMU.h"

double GetDiffTime(GPSTIME& first, GPSTIME& now) {
    return (now.Week - first.Week) * 604800.0 + now.SOW - first.SOW;
}

void CalculateCbn(AlignModule& align) {
    Vector3d gravity, omega;
    for (int j = 0; j < 3; ++j) {
        gravity[j] = -align.fmean[j];
        omega[j] = align.anglevelocitymean[j];
    }
    const Vector3d cross = gravity.cross(omega);
    if (!gravity.allFinite() || !omega.allFinite() || gravity.norm() < 1e-12 ||
        cross.norm() < 1e-12 || abs(cos(Latitude)) < 1e-12)
        throw runtime_error("Degenerate vectors in static alignment");
    const Vector3d down = gravity.normalized();
    const Vector3d east = cross.normalized();
    const Vector3d north = east.cross(down).normalized();
    align.Cbn.row(0) = north.transpose();
    align.Cbn.row(1) = east.transpose();
    align.Cbn.row(2) = down.transpose();
    align.AttitudeAngle(); // 内部弧度；输出列明确转为度
}

namespace {
struct DifferenceStats {
    Vector3d squares = Vector3d::Zero(), maximum = Vector3d::Zero();
    size_t n = 0;
    void add(const AlignModule& now, const AlignModule& ref) {
        Vector3d error(now.yaw - ref.yaw, now.pitch - ref.pitch, now.roll - ref.roll);
        for (int j = 0; j < 3; ++j) {
            error[j] = remainder(error[j], 2 * PI) * rad2deg;
            squares[j] += error[j] * error[j];
            maximum[j] = max(maximum[j], abs(error[j]));
        }
        ++n;
    }
    void write(ofstream& out) const {
        if (!n) throw runtime_error("No alignment solutions");
        out << "%Count=" << n << "\n%RMS_difference_to_whole(deg):";
        for (int j = 0; j < 3; ++j) out << (j ? "," : "") << sqrt(squares[j] / n);
        out << "\n%Max_abs_difference_to_whole(deg):";
        for (int j = 0; j < 3; ++j) out << (j ? "," : "") << maximum[j];
        out << '\n';
    }
};

void Angles(ofstream& out, const AlignModule& align) {
    out << fixed << setprecision(12) << align.yaw * rad2deg << ','
        << align.pitch * rad2deg << ',' << align.roll * rad2deg;
}

void Add(const ASCData& row, Vector3d& force, Vector3d& omega) {
    for (int j = 0; j < 3; ++j) {
        force[j] += row.acce.acce[j];
        omega[j] += row.gyro.gyro[j];
    }
}

void SolveMeanAttitude(const Vector3d& force, const Vector3d& omega, size_t count, AlignModule& align) {
    if (!count) throw runtime_error("Empty alignment interval");
    for (int j = 0; j < 3; ++j) {
        align.fmean[j] = force[j] / count;
        align.anglevelocitymean[j] = omega[j] / count;
    }
    CalculateCbn(align);
}
}

void EpochAlign(ifstream& file, ASCData, AlignModule& now, AlignModule& total, ofstream& out) {
    DifferenceStats stats;
    out << "%Time_sow(s),Yaw,Pitch,Roll(deg)\n";
    for (const auto& row : ReadRecords(file, ALIGNMENT)) {
        Vector3d force = Vector3d::Zero(), omega = Vector3d::Zero();
        Add(row, force, omega);
        now.time = row.time;
        SolveMeanAttitude(force, omega, 1, now);
        stats.add(now, total);
        out << fixed << setprecision(9) << now.time.SOW << ',';
        Angles(out, now);
        out << '\n';
    }
    stats.write(out);
}

void SecondAlign(ifstream& file, ASCData, AlignModule& now, AlignModule& total, ofstream& out) {
    const auto rows = ReadRecords(file, ALIGNMENT);
    GPSTIME origin = rows.front().time;
    Vector3d force = Vector3d::Zero(), omega = Vector3d::Zero();
    size_t count = 0;
    int bin = 0;
    DifferenceStats stats;
    out << "%Window_center_relative(s),Yaw,Pitch,Roll(deg),Samples\n";
    auto flush = [&]() {
        // 允许少量缺测；严重缺测和末尾不足1秒的窗口不计入每秒结果。
        if (count >= static_cast<size_t>(FNSC) - 2) {
            SolveMeanAttitude(force, omega, count, now);
            stats.add(now, total);
            out << fixed << setprecision(3) << bin + 0.5 << ',';
            Angles(out, now);
            out << ',' << count << '\n';
        }
    };
    for (const auto& row : rows) {
        GPSTIME time = row.time;
        const int current = static_cast<int>(floor((GetDiffTime(origin, time) + 1e-7)));
        if (current != bin) {
            flush();
            bin = current;
            count = 0;
            force.setZero();
            omega.setZero();
        }
        // 边界样本进入下一秒，绝不在刷新时丢掉。
        Add(row, force, omega);
        now.time = row.time;
        ++count;
    }
    if (GetDiffTime(origin, now.time) + 1.0 / FNSC >= bin + 1.0 - 1e-7) flush();
    stats.write(out);
}

void AllAlign(ifstream& file, ASCData& data, AlignModule& align, ofstream& out) {
    data.ft = ALIGNMENT;
    const auto rows = ReadRecords(file, ALIGNMENT);
    Vector3d force = Vector3d::Zero(), omega = Vector3d::Zero();
    for (const auto& row : rows) Add(row, force, omega);
    SolveMeanAttitude(force, omega, rows.size(), align);
    out << "%Yaw,Pitch,Roll(deg)\n";
    Angles(out, align);
    out << "\n%Count=" << rows.size() << "\n%C_body_to_NED\n" << align.Cbn
        << "\n%Orthogonality_F=" << (align.Cbn * align.Cbn.transpose() - Matrix3d::Identity()).norm()
        << "\n%Determinant=" << align.Cbn.determinant() << '\n';
}

void CalTimeNoise(ifstream& file, ASCData& data, ofstream& out) {
    data.ft = ALIGNMENT;
    const auto rows = ReadRecords(file, ALIGNMENT);
    // 陀螺零偏0.05 deg/h；ARW 0.005 deg/sqrt(h)。输出是航向误差量级，单位deg。
    const double earthHorizontalDegH = OMEGA * rad2deg * 3600 * cos(Latitude);
    out << "%Bias_heading(deg):" << fixed << setprecision(12)
        << 0.05 / earthHorizontalDegH * rad2deg << "\n%Time(s),WhiteNoise_heading(deg)\n";
    GPSTIME origin = rows.front().time;
    for (size_t i = 1; i < rows.size(); ++i) {
        GPSTIME time = rows[i].time;
        const double seconds = GetDiffTime(origin, time);
        const double noise = (0.005 / sqrt(seconds / 3600.0)) / earthHorizontalDegH * rad2deg;
        out << setprecision(6) << seconds << ',' << setprecision(12) << noise << '\n';
    }
}
