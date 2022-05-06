/*** 
 * @Author              : Fantongwen
 * @Date                : 2022-05-03 20:31:46
 * @LastEditTime        : 2022-05-06 17:09:59
 * @LastEditors         : Fantongwen
 * @Description         : 测试主函数入口
 * @FilePath            : \IndoorLooseComb\testmech.cpp
 * @Copyright (c) 2021
 */

#include "utils.h"
#include "mech.h"
#include <fstream>
#include <iostream>
#include "ekf.h"
int main1(int argc, char** argv)
{
    std::ifstream fimudata("F:\\affairs\\courses\\NavSystemDesign\\uwb_20220429\\02\\HL_INSPROBE_1_VEL_IMU.txt", std::ios_base::in);
    IMUDATA_T imu_data;
    ReadIMUdata(fimudata, imu_data);

    PVASTATE_T pva_state_0;
    pva_state_0.timestamp = imu_data.timestamp;
    pva_state_0.p_n << 0, 0, 0;
    pva_state_0.v_n << 0, 0, 0;
    pva_state_0.e_bn << 0, 0, 0;
    pva_state_0.q_bn = Euler2Quart(pva_state_0.e_bn);
    pva_state_0.c_bn = pva_state_0.q_bn.toRotationMatrix();

    mech *mechtest;
    mechtest = new mech(pva_state_0);

    while(!fimudata.eof())
    {
        ReadIMUdata(fimudata, imu_data);
        mechtest->mech_updatatick(imu_data);
    }


    fimudata.close();
    return 0;
}