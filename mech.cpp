/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-04 09:57:41
 * @LastEditTime        : 2022-05-04 19:02:57
 * @LastEditors         : Fantongwen
 * @Description         : 
 * @FilePath            : \IndoorLooseComb\mech.cpp
 * @Copyright (c) 2021
 */

#include "mech.h"
#include "utils.h"
#include <iostream>
#include <iomanip>
mech::mech(const PVASTATE_T &pva_state_0)
{
    pva_state = new PVASTATE_T(pva_state_0);
    fmechdata.open("..\\data\\mechdata.txt", std::ios_base::out);
}

mech::~mech()
{
    delete pva_state;

    pva_state = NULL;
    fmechdata.close();
}

void mech::mech_updatatick(const IMUDATA_T &imu_data)
{
    double dt;
    Eigen::Matrix3d c_bn_new;
    Eigen::Vector3d v_n_new;
    Eigen::Vector3d p_n_new;

    dt = imu_data.timestamp - pva_state->timestamp;
    // 姿态更新
    c_bn_new = pva_state->c_bn + pva_state->c_bn * Vector2CrossMatrix(imu_data.gyro_data * dt);
    // 速度更新
    v_n_new = pva_state->v_n + ((c_bn_new * imu_data.accel_data) - G_N) * dt;
    // 位置更新
    p_n_new = pva_state->p_n + v_n_new * dt;

    pva_state->timestamp = imu_data.timestamp;
    pva_state->c_bn = c_bn_new;
    pva_state->e_bn = DCM2Euler(c_bn_new);
    pva_state->q_bn = Euler2Quart(pva_state->e_bn);
    pva_state->v_n = v_n_new;
    pva_state->p_n = p_n_new;
    // log
    fmechdata << std::setprecision(16) <<  pva_state->timestamp << " " << 
        std::setprecision(16) <<  pva_state->p_n[X_B] << " " << 
        std::setprecision(16) <<  pva_state->p_n[Y_B] << " " << 
        std::setprecision(16) <<  pva_state->p_n[Z_B] << " " << 
        std::setprecision(16) <<  pva_state->v_n[X_B] << " " << 
        std::setprecision(16) <<  pva_state->v_n[Y_B] << " " << 
        std::setprecision(16) <<  pva_state->v_n[Z_B] << " " << 
        std::setprecision(16) <<  pva_state->e_bn[YAW] * R2D << " " << 
        std::setprecision(16) <<  pva_state->e_bn[PITCH] * R2D << " " << 
        std::setprecision(16) <<  pva_state->e_bn[ROLL] * R2D << " " << 
        std::setprecision(16) <<  dt << " " << std::endl;

    static int mech_log_index = 0;
    if (mech_log_index < 1000)
    {
        mech_log_index++;
    }
    else
    {
        mech_log_index = 0;
        std::cout << pva_state->timestamp << " " << 
        pva_state->e_bn[YAW] * R2D << " " << 
        pva_state->e_bn[PITCH] * R2D << " " << 
        pva_state->e_bn[ROLL] * R2D << std::endl;
    }
}