/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-03 20:29:27
 * @LastEditTime        : 2022-05-07 17:04:36
 * @LastEditors         : Fantongwen
 * @Description         : 需要的工具函数,和数据结构
 * @FilePath            : \IndoorLooseComb\utils.h
 * @Copyright (c) 2021
 */
#pragma once
#include "Eigen/Dense"
#include "fstream"
#include <cmath>
#include <iostream>

// 载体坐标系
typedef enum EN_BFRAME
{
    X_B,
    Y_B,
    Z_B
} BFRAME_E;

// 载体坐标系
typedef enum EN_EULLERANGLE
{
    YAW,
    PITCH,
    ROLL
} EULLERANGLE_E;

typedef struct ST_IMUDATA
{
    double timestamp;
    Eigen::Vector3d accel_data;
    Eigen::Vector3d gyro_data;
} IMUDATA_T;

typedef struct ST_ODOMDATA
{
    double timestamp;
    Eigen::Vector3d vel_data;
    bool valid;
} ODOMDATA_T;

typedef struct ST_EKFDATA
{
    IMUDATA_T imu_data;
    ODOMDATA_T odom_data;
} EKFDATA_T;

typedef struct ST_PVASTATE
{
    double timestamp;
    Eigen::Vector3d p_n;
    Eigen::Vector3d v_n;
    Eigen::Matrix3d c_bn;
    Eigen::Vector3d e_bn;
    Eigen::Quaterniond q_bn;
    ST_PVASTATE()
    {
        ;
    }
    ST_PVASTATE(const ST_PVASTATE &pva_state_0)
    {
        timestamp = pva_state_0.timestamp;
        p_n = pva_state_0.p_n;
        v_n = pva_state_0.v_n;
        c_bn = pva_state_0.c_bn;
        e_bn = pva_state_0.e_bn;
        q_bn = pva_state_0.q_bn;
    }
} PVASTATE_T;

typedef struct ST_EKF_MAT
{
    Eigen::Matrix<double, 16, 1> delta_x;
    Eigen::Matrix<double, 16, 16> Phi_mat;

    Eigen::Matrix<double, 16, 13> G_mat;
    Eigen::Matrix<double, 13, 13> q_mat;
    Eigen::Matrix<double, 16, 16> Q_mat;

    Eigen::Matrix<double, 3, 16> H_mat;
    Eigen::Matrix<double, 16, 16> P_mat;
    Eigen::Matrix<double, 16, 3> K_mat;
    Eigen::Matrix<double, 3, 3> R_mat;
    Eigen::Matrix<double, 3, 1> delta_z;

    ST_EKF_MAT() : delta_x(Eigen::Matrix<double, 16, 1>::Zero()),
                   Phi_mat(Eigen::Matrix<double, 16, 16>::Zero()),
                   G_mat(Eigen::Matrix<double, 16, 13>::Zero()),
                   q_mat(Eigen::Matrix<double, 13, 13>::Zero()),
                   Q_mat(Eigen::Matrix<double, 16, 16>::Zero()),
                   H_mat(Eigen::Matrix<double, 3, 16>::Zero()),
                   P_mat(Eigen::Matrix<double, 16, 16>::Zero()),
                   K_mat(Eigen::Matrix<double, 16, 3>::Zero()),
                   R_mat(Eigen::Matrix<double, 3, 3>::Zero())
    {
        ;
    }
    void ekf_predict()
    {
        delta_x = Phi_mat * delta_x;
        P_mat = Phi_mat * P_mat * Phi_mat.transpose() + Q_mat;
    }

    bool ekf_measurement()
    {
        Eigen::Matrix<double, 16, 16> I_16;
        I_16.setIdentity();
        Eigen::Matrix3d C_mat = (H_mat * P_mat * H_mat.transpose() + R_mat);
        Eigen::Vector3d INNO = (delta_z - H_mat * delta_x);
        if (INNO[0]>36*C_mat(0,0) || 
        INNO[1]>36*C_mat(1,1) || 
        INNO[2]>36*C_mat(2,2))
        {
            std::cout << "inno " << INNO.transpose() << std::endl;
            return false;
        }
        K_mat = P_mat * H_mat.transpose() * ((H_mat * P_mat * H_mat.transpose() + R_mat).inverse());
        delta_x = delta_x + K_mat * (delta_z - H_mat * delta_x);
        P_mat = (I_16 - (K_mat * H_mat)) * P_mat * (I_16 - (K_mat * H_mat)).transpose() + K_mat * R_mat * K_mat.transpose();
        return true;
    }
} EKF_MAT_T;

// 里程计状态
typedef struct ST_ODOM_INFO
{
    Eigen::Vector3d e_bv; // 安装欧拉角 b系到v系
    Eigen::Matrix3d c_bv; // b系到v系旋转矩阵
    Eigen::Vector3d l_bv; // 小车中心相对于imu杆臂
} ODOM_INFO_T;

// 传感器参数
typedef struct ST_SENSOR_PARAM
{
    Eigen::Vector3d odom_scale_factor;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d accel_bias;
} SENSOR_PARAM_T;

// 里程计降采样
typedef struct ST_ODOMDATA_DOWNSAMPLE
{
    double timestamp;
    double T_threshold;
    Eigen::Vector3d odom_Distance;
    double odom_Time;
    // 初始化
    ST_ODOMDATA_DOWNSAMPLE(
        const double &t_init, 
        const double &thres_init):
        timestamp(t_init),
        T_threshold(thres_init),
        odom_Distance(Eigen::Vector3d::Zero()),
        odom_Time(0.0)
    {
        ;
    }
    // downsample 
    ODOMDATA_T downsample_updata(const ODOMDATA_T& odom_data)
    {
        ODOMDATA_T odom_data_ds = odom_data;
        odom_data_ds.valid = false;
        double dt;
        dt = odom_data.timestamp - timestamp;
        odom_Time += dt;
        odom_Distance += dt * odom_data.vel_data;
        if (odom_Time >= T_threshold)
        {
            // 时间超过门限,给出降采样后的里程计数据
            odom_data_ds.vel_data = odom_Distance / odom_Time;
            odom_data_ds.valid = true;
            // 状态复位
            odom_Distance.setZero();
            odom_Time = 0.0;
        }

        timestamp = odom_data.timestamp;
        return odom_data_ds;
    }

} ODOMDATA_DOWNSAMPLE_T;

const Eigen::Vector3d G_N = {0, 0, -9.7936};
const double R2D = (180.0 / M_PI);
const double D2R = (M_PI / 180.0);
const double HOUR2S = 3600.0;
const double SQRT_HOUR2S = 60.0;
const double MGAL = 1.0E-5;
// IMU参数
const double VRW = 0.1 / SQRT_HOUR2S;
const double ARW = 0.1 * D2R / SQRT_HOUR2S;
const double GYRO_BIAS_STD = 50 * D2R / HOUR2S;
const double GYRO_BIAS_CORR_TIME = 1 * HOUR2S;
const double ACCEL_BIAS_STD = 50 * MGAL;
const double ACCEL_BIAS_CORR_TIME = 1 * HOUR2S;
// ODOM参数
const double ODOM_STD = 0.05;
const double ODOM_SCALE_STD = 0.00001;

const double ODOMDATA_INTERVAL_SET = 0.1;

Eigen::Matrix3d Vector2CrossMatrix(const Eigen::Vector3d &a);
Eigen::Vector3d DCM2Euler(const Eigen::Matrix3d &dcm);
Eigen::Quaterniond Euler2Quart(const Eigen::Vector3d &vec);

void ReadIMUdata(std::ifstream &fimudata, IMUDATA_T &imu_data);
void ReakEKFdata(std::ifstream &fimudata, EKFDATA_T &ekf_data);