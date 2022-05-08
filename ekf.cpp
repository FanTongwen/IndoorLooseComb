/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-05 16:40:48
 * @LastEditTime        : 2022-05-08 10:45:38
 * @LastEditors         : Fantongwen
 * @Description         : 
 * @FilePath            : \IndoorLooseComb\ekf.cpp
 * @Copyright (c) 2021
 */

#include "ekf.h"
#include <iomanip>

// TODO: 为指针分配内存并初始化 打开文件
ekf::ekf(
            const PVASTATE_T &pva_state_0, 
            const EKFDATA_T &ekf_data, 
            const ODOM_INFO_T &odom_info, 
            const SENSOR_PARAM_T &sensor_param)
{
    // mech
    ekf_mech = new mech(pva_state_0);
    // ekf_mat
    ekf_mat = new EKF_MAT_T();
    Eigen::Matrix<double, 16, 16> P_mat;
    P_mat.setZero();
    Eigen::Matrix3d I_3_3;
    I_3_3.setIdentity();
    P_mat.block<3, 3>(9, 9) = I_3_3 * GYRO_BIAS_STD * GYRO_BIAS_STD;
    P_mat.block<3, 3>(12, 12) = I_3_3 * ACCEL_BIAS_STD * ACCEL_BIAS_STD;
    P_mat(15, 15) = 0;
    ekf_mat->P_mat = P_mat;
    Eigen::Matrix<double, 16, 13> G_mat;
    G_mat.setZero();
    G_mat.block<3, 3>(3, 0) = pva_state_0.c_bn;
    G_mat.block<3, 3>(6, 3) = pva_state_0.c_bn;
    G_mat.block<7, 7>(9, 6).setIdentity();
    ekf_mat->G_mat = G_mat;
    // ekf_sensor_param
    ekf_sensor_param = new SENSOR_PARAM_T(sensor_param);
    // ekf_odom_info
    ekf_odom_info = new ODOM_INFO_T(odom_info);
    // ekfdata_current ekfdata_last
    ekfdata_current = new EKFDATA_T();
    ekfdata_last = new EKFDATA_T();
    EKFdataCompensate(ekf_data);
    *ekfdata_last = *ekfdata_current;
    // ekf_odom_ds
    ekf_odom_ds = new ODOMDATA_DOWNSAMPLE_T(ekf_data.odom_data.timestamp, ODOMDATA_INTERVAL_SET);
    // fekfdata
    fekfdata.open("F:\\affairs\\courses\\NavSystemDesign\\UWB_LooseComb\\IndoorLooseComb\\data\\ekfmid.txt", std::ios::out);
}

// TODO: 回收内存 指针指向空 关闭文件
ekf::~ekf()
{
    delete ekf_mech;
    ekf_mech = NULL;
    delete ekf_mat;
    ekf_mat = NULL;
    delete ekfdata_current;
    ekfdata_current = NULL;
    delete ekfdata_last;
    ekfdata_last = NULL;
    delete ekf_odom_info;
    ekf_odom_info = NULL;
    delete ekf_sensor_param;
    ekf_sensor_param = NULL;

    fekfdata.close();
}

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
    if (delta_t < 1.0)
    {
        ekf_mat->Phi_mat = I_16_16 + F_mat * delta_t;
    }
    else
    {
        ekf_mat->Phi_mat = I_16_16 + F_mat * 0.005;
    }
    
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
    q_mat(12, 12) =  ODOM_SCALE_STD * ODOM_SCALE_STD; // NOTE: 这里怎么给里程计零偏
    ekf_mat->q_mat = q_mat;
    // 求Q_mat
    Eigen::Matrix<double, 16, 16> Q_mat;
    if (delta_t < 1.0)
    {
        Q_mat = 0.5 * (ekf_mat->Phi_mat * G_mat_last * q_mat * G_mat_last.transpose() * ekf_mat->Phi_mat.transpose() + G_mat * q_mat * G_mat.transpose()) * delta_t;
    }
    else
    {
        Q_mat = 0.5 * (ekf_mat->Phi_mat * G_mat_last * q_mat * G_mat_last.transpose() * ekf_mat->Phi_mat.transpose() + G_mat * q_mat * G_mat.transpose()) * 0.005;
    }
    ekf_mat->Q_mat = Q_mat;
    ekf_mat->ekf_predict();
    pva_state = NULL;
}

bool ekf::EKFmeasurementUpdate(const double &delta_t)
{
    Eigen::Matrix3d I_3_3;
    I_3_3.setIdentity();
    const PVASTATE_T *pva_state = ekf_mech->pva_state;
    const ODOMDATA_T *odom_data = &(ekfdata_current->odom_data);
    const IMUDATA_T *imu_data = &(ekfdata_current->imu_data);

    Eigen::Matrix3d c_nb;
    c_nb = pva_state->c_bn.transpose();
    Eigen::Matrix3d c_bv;
    c_bv = ekf_odom_info->c_bv;

    // 求观测向量delta_z
    Eigen::Vector3d delta_z;
    Eigen::Vector3d v_v_odom = odom_data->vel_data;
    Eigen::Vector3d v_v_ins = c_bv * c_nb * pva_state->v_n - 
    c_bv * Vector2CrossMatrix(ekf_odom_info->l_bv) * imu_data->gyro_data; // NOTE: 小影响
    delta_z = v_v_ins - v_v_odom;
    ekf_mat->delta_z = delta_z;

    // 求观测矩阵 H_mat
    Eigen::Vector3d I_3_1;
    I_3_1.setOnes();
    Eigen::Matrix<double, 3, 16> H_mat;
    H_mat.setZero();

    H_mat.block<3, 3>(0, 3) = c_bv * c_nb;
    H_mat.block<3, 3>(0, 6) = -c_bv * c_nb * Vector2CrossMatrix(pva_state->v_n);
    H_mat.block<3, 3>(0, 9) = -c_bv * Vector2CrossMatrix(ekf_odom_info->l_bv);
    H_mat.block<3, 1>(0, 15) = -v_v_odom / (1.0 + ekf_mat->delta_x(15, 0)); // -v_v_odom[0] * I_3_1 NOTE: 细微影响
    ekf_mat->H_mat = H_mat;
    // 求观测协方差矩阵 R_mat
    ekf_mat->R_mat = ODOM_STD * ODOM_STD * I_3_3;
    // ekf量测更新公式
    return ekf_mat->ekf_measurement();
}

// 量测更新完成之后进行的误差校正
void ekf::EKFcorrectUpdate()
{
    // pva 误差校正
    ekf_mech->pva_state->p_n -= ekf_mat->delta_x.block<3, 1>(0, 0);
    ekf_mech->pva_state->v_n -= ekf_mat->delta_x.block<3, 1>(3, 0);
    Eigen::Vector3d phi = ekf_mat->delta_x.block<3, 1>(6, 0);
    Eigen::Quaterniond q_np;
    Eigen::Quaterniond q_bn;
    q_np.w() = cos((0.5 * phi).norm());
    q_np.vec() = sin((0.5 * phi).norm()) / ((0.5 * phi).norm()) * 0.5 * phi;
    q_bn = q_np * ekf_mech->pva_state->q_bn;
    q_bn.normalize();
    ekf_mech->pva_state->q_bn = q_bn;
    ekf_mech->pva_state->c_bn = q_bn.matrix();
    ekf_mech->pva_state->e_bn = DCM2Euler(ekf_mech->pva_state->c_bn);

    // sensor_param 误差校正
    ekf_sensor_param->gyro_bias += ekf_mat->delta_x.block<3, 1>(9, 0);
    ekf_sensor_param->accel_bias += ekf_mat->delta_x.block<3, 1>(12, 0);
    ekf_sensor_param->odom_scale_factor(0) += ekf_mat->delta_x(15);
    // 状态量清零
    ekf_mat->delta_x.setZero();
}

// 对传感器数据进行补偿
void ekf::EKFdataCompensate(const EKFDATA_T &ekf_data)
{
    IMUDATA_T imudata;
    ODOMDATA_T odomdata;

    imudata.timestamp = ekf_data.imu_data.timestamp;
    imudata.gyro_data = ekf_data.imu_data.gyro_data - ekf_sensor_param->gyro_bias;
    imudata.accel_data = ekf_data.imu_data.accel_data - ekf_sensor_param->accel_bias;

    odomdata.timestamp = ekf_data.odom_data.timestamp;
    odomdata.vel_data = (ekf_data.odom_data.vel_data.array() / 
    (Eigen::Vector3d::Ones() + ekf_sensor_param->odom_scale_factor).array()).matrix();
    odomdata.valid = ekf_data.odom_data.valid;

    ekfdata_current->imu_data = imudata;
    ekfdata_current->odom_data = odomdata;
}

// ekf更新总函数
void ekf::EKFUpdate(const EKFDATA_T &ekf_data, int &odom_update_flag)
{
    odom_update_flag = 0;
    double dt;
    dt = ekf_data.imu_data.timestamp - ekfdata_current->imu_data.timestamp;
    ODOMDATA_T odom_data_ds;
    odom_data_ds = ekf_odom_ds->downsample_updata(ekf_data.odom_data);
    // odom时间戳小于上一时刻imu 需要更新odom数据
    if(ekf_data.odom_data.timestamp >= ekfdata_last->imu_data.timestamp
    && ekf_data.odom_data.timestamp <= ekf_data.imu_data.timestamp
    && odom_data_ds.valid)
    {
        EKFDATA_T ekf_data_ds;
        ekf_data_ds = ekf_data;
        ekf_data_ds.odom_data = odom_data_ds;
        // 量测
        double dt1, dt2;
        dt1 = ekf_data.odom_data.timestamp - ekfdata_last->imu_data.timestamp;
        dt2 = ekf_data.imu_data.timestamp - ekf_data.odom_data.timestamp;
        if (dt1 < dt2)
        {
            // 上一刻的时间差
            dt = ekfdata_current->imu_data.timestamp - ekfdata_last->imu_data.timestamp;
            if(EKFmeasurementUpdate(dt))
                EKFcorrectUpdate();
            // 此刻的时间差
            dt = ekf_data.imu_data.timestamp - ekfdata_current->imu_data.timestamp;
            EKFdataCompensate(ekf_data_ds); // 补偿同时更新了ekfdata_current
            ekf_mech->mech_updatatick(ekfdata_current->imu_data);
            EKFpredictUpdate(dt);
        }
        else
        {
            dt = ekf_data.imu_data.timestamp - ekfdata_current->imu_data.timestamp;
            EKFdataCompensate(ekf_data_ds); // 补偿同时更新了ekfdata_current
            ekf_mech->mech_updatatick(ekfdata_current->imu_data);
            EKFpredictUpdate(dt);
            if(EKFmeasurementUpdate(dt))
                EKFcorrectUpdate();
        }
        odom_update_flag = 1; // 需要读下一个odom的数据
    }
    else
    {
        if (ekf_data.odom_data.timestamp < ekfdata_last->imu_data.timestamp)
        {
            odom_update_flag = 1;
        }
        if (ekf_data.odom_data.timestamp >= ekfdata_last->imu_data.timestamp
            && ekf_data.odom_data.timestamp <= ekf_data.imu_data.timestamp)
        {
            odom_update_flag = 1;
        }
        
        // 机械编排
        dt = ekf_data.imu_data.timestamp - ekfdata_current->imu_data.timestamp;
        EKFdataCompensate(ekf_data);
        ekf_mech->mech_updatatick(ekfdata_current->imu_data);
        EKFpredictUpdate(dt);
    }
    *ekfdata_last = *ekfdata_current;

    fekfdata << std::setprecision(16) << ekf_data.imu_data.timestamp
             << " " << std::setprecision(16) << ekf_sensor_param->gyro_bias[0] << " " << std::setprecision(16) << ekf_sensor_param->gyro_bias[1] << " " << std::setprecision(16) << ekf_sensor_param->gyro_bias[2]
             << " " << std::setprecision(16) << ekf_sensor_param->accel_bias[0] << " " << std::setprecision(16) << ekf_sensor_param->accel_bias[1] << " " << std::setprecision(16) << ekf_sensor_param->accel_bias[2]
             << " " << std::setprecision(16) << ekf_sensor_param->odom_scale_factor[0] << " " << std::setprecision(16) << ekf_sensor_param->odom_scale_factor[1] << " " << std::setprecision(16) << ekf_sensor_param->odom_scale_factor[2]
             << " " << std::setprecision(16) << ekf_mat->delta_z[0] << " " << std::setprecision(16) << ekf_mat->delta_z[1] << " " << std::setprecision(16) << ekf_mat->delta_z[2]
             << " " << std::endl;
}