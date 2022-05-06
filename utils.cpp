/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-04 10:29:16
 * @LastEditTime        : 2022-05-04 19:45:12
 * @LastEditors         : Fantongwen
 * @Description         : 
 * @FilePath            : \IndoorLooseComb\utils.cpp
 * @Copyright (c) 2021
 */

#include "utils.h"

Eigen::Matrix3d Vector2CrossMatrix(const Eigen::Vector3d &a)
{
    Eigen::Matrix3d a_crossMatrix;
    a_crossMatrix << 0, -a(2), a(1),
        a(2), 0, -a(0),
        -a(1), a(0), 0;
    return a_crossMatrix;
}

Eigen::Vector3d DCM2Euler(const Eigen::Matrix3d &dcm)
{
    Eigen::Vector3d vec;
    vec << atan2(dcm(1, 0), dcm(0, 0)),
        atan2(-dcm(2, 0), sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2))),
        atan2(dcm(2, 1), dcm(2, 2));
    return vec;
}

Eigen::Quaterniond Euler2Quart(const Eigen::Vector3d &vec)
{
    Eigen::Quaterniond quat;
    Eigen::AngleAxisd rollAngle(vec(ROLL), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(vec(PITCH), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(vec(YAW), Eigen::Vector3d::UnitZ());
    quat = yawAngle * pitchAngle * rollAngle;
    return quat;
}

void ReadIMUdata(std::ifstream &fimudata, IMUDATA_T &imu_data)
{
    double gx, gy, gz, ax, ay, az;
    fimudata >> imu_data.timestamp >> gx >> gy >> gz
    >> ax >> ay >> az;
    imu_data.gyro_data << -gx, -gy, gz;
    imu_data.accel_data << -ax, -ay, az;
}

void ReadIMUdata_odom(std::ifstream &fimudata, IMUDATA_T &imu_data)
{
    double gx, gy, gz, ax, ay, az, temp;
    fimudata >> imu_data.timestamp >> gx >> gy >> gz
    >> ax >> ay >> az >> temp >> temp;
    imu_data.gyro_data << gx, gy, gz;
    imu_data.accel_data << ax, ay, az;
}