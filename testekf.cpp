/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-06 16:31:21
 * @LastEditTime        : 2022-05-07 16:59:39
 * @LastEditors         : Fantongwen
 * @Description         : 
 * @FilePath            : \IndoorLooseComb\testekf.cpp
 * @Copyright (c) 2021
 */

#include "ekf.h"
#include "utils.h"
#include <iostream>

int main(int argc, char** argv)
{
    auto startTime  = 182815.0;
    auto endTime    = 184758.0;

    EKFDATA_T ekfdata;

    std::ifstream data_fs(
        "F:\\affairs\\courses\\NavSystemDesign\\robot_imudata\\HL_INSPROBE_9_VEL_IMU_ODO.txt");
    ReakEKFdata(data_fs, ekfdata);

    PVASTATE_T pvastate_0;
    pvastate_0.timestamp = startTime;
    pvastate_0.p_n << 0, 0, 0;
    pvastate_0.v_n << 0, 0, 0;
    pvastate_0.e_bn << 0, 0.109 * D2R, -0.23 * D2R;
    pvastate_0.q_bn = Euler2Quart(pvastate_0.e_bn);
    pvastate_0.c_bn = pvastate_0.q_bn.matrix();

    ODOM_INFO_T odominfo_0;
    odominfo_0.e_bv << -0.31 * D2R, -0.3 * D2R, 0;
    odominfo_0.c_bv = (Euler2Quart(odominfo_0.e_bv)).matrix();
    odominfo_0.l_bv << 0, 0, 1.099;

    SENSOR_PARAM_T sensorparam_0;
    sensorparam_0.gyro_bias << 3.4479949e-4, -1.034033e-03, 8.97390124e-05;
    sensorparam_0.accel_bias << 104.4e-5, -111.74e-5, 314.06e-5;
    sensorparam_0.odom_scale_factor << 0.01, 0, 0;

    ekf *ekftest;
    ekftest = new ekf(pvastate_0, ekfdata, odominfo_0, sensorparam_0);
    int flag = 0;
    while (!data_fs.eof()){
        ReakEKFdata(data_fs, ekfdata);
        if (ekfdata.imu_data.timestamp > startTime && ekfdata.imu_data.timestamp < endTime)
        {
            ekftest->EKFUpdate(ekfdata, flag);
        }
    }
    return 0;
}