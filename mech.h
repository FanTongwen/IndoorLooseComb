/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-03 20:24:46
 * @LastEditTime        : 2022-05-05 16:23:32
 * @LastEditors         : Fantongwen
 * @Description         : 室内机械编排,较为粗糙
 * @FilePath            : \IndoorLooseComb\mech.h
 * @Copyright (c) 2021
 */
#pragma once
#include "Eigen/Dense"
#include "utils.h"
#include <fstream>
class mech
{
    public:
        mech(const PVASTATE_T &pva_state_0);
        ~mech();
    public:
        PVASTATE_T *pva_state;
        std::ofstream fmechdata;

    public:
        void mech_updatatick(const IMUDATA_T &imu_data);
};