/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-05 14:02:15
 * @LastEditTime        : 2022-05-05 20:50:55
 * @LastEditors         : Fantongwen
 * @Description         : 
 * @FilePath            : \IndoorLooseComb\ekf.h
 * @Copyright (c) 2021
 */
#pragma once
#include "mech.h"
#include <fstream>
#include "Eigen/Dense"
#include "utils.h"
class ekf
{
    public:
        ekf();
        ~ekf();
    public:
        mech *ekf_mech;
        EKF_MAT_T *ekf_mat;
        EKFDATA_T *ekfdata_current;
        std::ofstream fekfdata;
    public:
        void EKFpredictUpdate(const double &delta_t);
        void EkfmeasurementUpdate(const double &delta_t);
        void EKFUpdate();
};