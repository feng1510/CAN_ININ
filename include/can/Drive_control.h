//
// Created by inin on 19-5-8.
//

#ifndef IN2_CONTROL_DRIVE_CONTROL_H
#define IN2_CONTROL_DRIVE_CONTROL_H

#include "define.h"
#include <can/VCU.h>
#include <can/ELSE.h>
#include <can/BCS.h>

using namespace std;
using namespace cv;

class Drive_control {

public:
    Drive_control();

    /************** SCU_1 0x37A************/
    //本地回环解码用
    struct can_frame frame;
    __u8 SCU_RemoteKeySt;             //赋值为3，请求进入自动驾驶
    __u8 SCU_EPBModeReq;              //不懂？？？？
    int16_t SCU_EmergencyStopReq;     //0：CDD制动，   1：AEB紧急制动
    float SCU_BrakeReq;             //-resolution: 0.1 % per bit-offset: 0
    float SCU_AutoTrqWhlReq;        //-resolution: 1.5 Nm per bit-offset: -3000

    __u8 SCU_DrvModeReq;             //0=Reserved    1=Manual    2=Remote    3=Auto

    //接收遥控指令
    float BrakeReq;               //0~99
    float AutoTrqWhlReq;          //-resolution: 1.5 Nm per bit-offset: -3000
    int16_t GearReq;                //档位0是空档，1是前进，2是倒档
    int16_t EmergencyStop;          //单独写一个急刹的请求

    __u8 RemoteKeySt;
    __u8 EPBModeReq;
    int16_t EmergencyStopReq;
    __u8 DrvModeReq;



    void drive_control_code();                         //编码
    void in_drive_auto(VCU& vcu, ELSE& elseid);        //进入自动模式
    void esc_drive_auto();                     //退出自动模式
    void drive_loop_decode(struct can_frame &frame);   //本地回环解码

    void drive_control_dis(BCS& bcs, VCU& vcu, ELSE &elseid, cv::Mat Display_control);   //显示
    void drive_emergencyStop(BCS& bcs);



private:



};


#endif //IN2_CONTROL_DRIVE_CONTROL_H
