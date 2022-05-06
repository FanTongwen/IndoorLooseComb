/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-05 14:02:15
 * @LastEditTime        : 2022-05-06 16:23:17
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
        ekf(
            const PVASTATE_T &pva_state_0, 
            const EKFDATA_T &ekf_data, 
            const ODOM_INFO_T &odom_info, 
            const SENSOR_PARAM_T &sensor_param);
        ~ekf();
    public:
        mech *ekf_mech;
        EKF_MAT_T *ekf_mat;
        EKFDATA_T *ekfdata_current;
        EKFDATA_T *ekfdata_last;
        ODOM_INFO_T *ekf_odom_info;
        SENSOR_PARAM_T *ekf_sensor_param;
        std::ofstream fekfdata;
    public:
        void EKFpredictUpdate(const double &delta_t);
        void EKFmeasurementUpdate(const double &delta_t);
        void EKFcorrectUpdate();
        void EKFdataCompensate(const EKFDATA_T &ekf_data);
        void EKFUpdate(const EKFDATA_T &ekf_data, int &odom_update_flag);
};