/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-05 16:40:48
 * @LastEditTime        : 2022-05-05 21:32:24
 * @LastEditors         : Fantongwen
 * @Description         : 
 * @FilePath            : \IndoorLooseComb\ekf.cpp
 * @Copyright (c) 2021
 */

#include "ekf.h"

void ekf::EKFpredictUpdate(const double &delta_t)
{
    Eigen::Matrix3d I_3_3;
    I_3_3.setIdentity();
    Eigen::Matrix<double, 16, 16> I_16_16;
    I_16_16.setIdentity();
    Eigen::Matrix<double, 16, 16> F_mat;
    F_mat.setZero();
    F_mat.block<3, 3>(0, 3) = I_3_3;
    Eigen::Vector3d a_n;
    const PVASTATE_T *pva_state = ekf_mech->pva_state;
    a_n = pva_state->c_bn * ekfdata_current->imu_data.accel_data;
    F_mat.block<3, 3>(3, 6) = Vector2CrossMatrix(a_n);
    F_mat.block<3, 3>(3, 12) = pva_state->c_bn;
    F_mat.block<3, 3>(6, 9) = -pva_state->c_bn;
    ekf_mat->Phi_mat = I_16_16 + F_mat * delta_t;
    // 求G_mat
    Eigen::Matrix<double, 16, 13> G_mat;
    Eigen::Matrix<double, 16, 13> G_mat_last;
    G_mat.setZero();
    G_mat.block<3, 3>(3, 0) = pva_state->c_bn;
    G_mat.block<3, 3>(6, 3) = pva_state->c_bn; 
    G_mat.block<3, 3>(9, 6) = I_3_3; 
    G_mat.block<3, 3>(12, 9) = I_3_3;
    G_mat(15, 12) = 1;
    G_mat_last = ekf_mat->G_mat;
    ekf_mat->G_mat = G_mat;
    // 求q_mat
    Eigen::Matrix<double, 13, 13> q_mat;
    q_mat.setZero();
    q_mat.block<3, 3>(0, 0) = VRW * VRW * I_3_3;
    q_mat.block<3, 3>(3, 3) = ARW * ARW * I_3_3;
    q_mat.block<3, 3>(6, 6) = 2 * GYRO_BIAS_STD * GYRO_BIAS_STD
    / GYRO_BIAS_CORR_TIME * I_3_3;
    q_mat.block<3, 3>(9, 9) = 2 * ACCEL_BIAS_STD * ACCEL_BIAS_STD
    / ACCEL_BIAS_CORR_TIME * I_3_3;
    q_mat(12, 12) = 2 * ODOM_SCALE_STD * ODOM_SCALE_STD; // NOTE: 这里怎么给里程计零偏
    ekf_mat->q_mat = q_mat;
    // 求Q_mat
    Eigen::Matrix<double, 16, 16> Q_mat;
    Q_mat = 0.5 * (ekf_mat->Phi_mat * G_mat_last * q_mat *
                   G_mat_last.transpose() * ekf_mat->Phi_mat.transpose() +
                   G_mat * q_mat * G_mat.transpose()) * delta_t;
    ekf_mat->Q_mat = Q_mat;
    ekf_mat->ekf_predict();
}

void ekf::EkfmeasurementUpdate(const double &delta_t)
{
    Eigen::Matrix3d I_3_3;
    I_3_3.setIdentity();

    // TODO: 求观测向量delta_z

    // TODO: 求观测矩阵 H_mat

    // 求观测协方差矩阵 R_mat
    ekf_mat->R_mat = ODOM_STD * ODOM_STD * I_3_3;
    // ekf量测更新公式
    ekf_mat->ekf_measurement();
}