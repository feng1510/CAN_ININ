//
// Created by inin on 19-5-8.
//

#include <iostream>
#include "can/Drive_control.h"


Drive_control::Drive_control() {

    SCU_RemoteKeySt = 3;    //允许VCU进入自动模式
    SCU_EPBModeReq = 0;
    SCU_EmergencyStopReq = 0;
    SCU_BrakeReq = 0;
    SCU_AutoTrqWhlReq = 0;
    SCU_DrvModeReq = 1;

    RemoteKeySt = 3;
    EPBModeReq = 0;
    EmergencyStopReq = 0;
    DrvModeReq = 1;
    BrakeReq = 0.0;
    AutoTrqWhlReq = 0.0;
    GearReq = 0;
    EmergencyStop = 0;


    frame.can_dlc = 8;
    frame.can_id = SCU_1;
    frame.data[0] = 0x03;
    frame.data[1] = 0x7D;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x7D;
    frame.data[6] = 0x00;
    frame.data[7] = 0x01;

}

//编码
void Drive_control::drive_control_code() {

    __u8 code = 0x00;
    int16_t TrqWhlReq_code = 0x0000;

    code = RemoteKeySt & 0x03;
    code = code | ((EPBModeReq & 0x03) << 2);
    code = code | ((EmergencyStopReq & 0x01) << 6);
    frame.data[0] = code;

    code = frame.data[3] & 0xFC;

    int16_t brake_code = 0x00;
    brake_code = int(BrakeReq * 10);
    frame.data[3] = code | ((brake_code >> 8) & 0x03);

    frame.data[4] = brake_code & 0xFF;                //Brake_Req接收0--100

    //如果现在是空档，则将扭矩赋值为0
    if(GearReq == 0) {
        AutoTrqWhlReq = 0;
    }

    TrqWhlReq_code = int((AutoTrqWhlReq + 3000)/1.5);     //-resolution: 1.5 Nm per bit-offset: -3000
    frame.data[5] = (TrqWhlReq_code >> 4) & 0x00FF;
    frame.data[6] = (TrqWhlReq_code << 4) & 0x00F0;
    frame.data[7] = DrvModeReq & 0x03;

}

//进入自动驾驶模式
void Drive_control::in_drive_auto(VCU &vcu, ELSE &elseid)  {
    if(vcu.VCU_DrvModeAct == 1 && vcu.VCU_CrntGearLvl == 2 && elseid.BCM_KeySt == 2) {

        frame.data[0] = 0x03;
        frame.data[1] = 0x7D;
        frame.data[2] = 0x00;
        frame.data[3] = 0x00;
        frame.data[4] = 0x00;
        frame.data[5] = 0x7D;
        frame.data[6] = 0x00;
        frame.data[7] = 0x03;
    }
}

//退出自动模式
void Drive_control::esc_drive_auto() {

    frame.can_dlc = 8;
    frame.can_id = SCU_1;
    frame.data[0] = 0x03;
    frame.data[1] = 0x7D;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x7D;
    frame.data[6] = 0x00;
    frame.data[7] = 0x01;

}

//急刹模式
void Drive_control::drive_emergencyStop(BCS &bcs) {

    //先只用CDD模式，后面增加AEB模式
    frame.data[0] = 0x03;
    frame.data[1] = 0x7D;
    frame.data[2] = 0x00;
    frame.data[3] = 0x03;
    frame.data[4] = 0xE8;
    frame.data[5] = 0x7D;
    frame.data[6] = 0x00;
    frame.data[7] = 0x03;
}



//本地回环解码
void Drive_control::drive_loop_decode(struct can_frame &frame) {

    __u16 decode = 0x0000;
    __u16 decode2 = 0x0000;

    SCU_RemoteKeySt = frame.data[0] & 0x03;
    SCU_EPBModeReq = (frame.data[0] >> 2 ) & 0x03;
    SCU_EmergencyStopReq = (frame.data[0] >> 6) & 0x01;

    decode = frame.data[3] & 0x03;
    decode = (decode << 8) | frame.data[4];
    SCU_BrakeReq = decode * 0.1;

    decode = 0x0000;
    decode = (frame.data[5] >> 4) & 0x0F;
    decode2 = ((frame.data[5] << 4) & 0xF0) | ((frame.data[6] >> 4) & 0x0F);
    decode2 = (decode << 8) | decode2;
    SCU_AutoTrqWhlReq = decode2 * 1.5 - 3000;

    SCU_DrvModeReq = frame.data[7] & 0x03;
}


//显示
void Drive_control::drive_control_dis(BCS& bcs, VCU &vcu, ELSE &elseid, cv::Mat Display_control) {

    char title_data[14][50];

    //本地回环
    switch(SCU_RemoteKeySt) {
        case 0:
            sprintf(title_data[0], "SCU_RemoteKeySt : Off");
            break;
        case 1:
            sprintf(title_data[0], "SCU_RemoteKeySt : Acc");
            break;
        case 2:
            sprintf(title_data[0], "SCU_RemoteKeySt : On");
            break;
        case 3:
            sprintf(title_data[0], "SCU_RemoteKeySt : Crank");
            break;
        default:
            sprintf(title_data[0], "SCU_RemoteKeySt : ERROR");
            break;
    }

    if(SCU_EmergencyStopReq == 1)
        sprintf(title_data[1], "SCU_EmergencyStopReq : Emergency Stop");
    else
        sprintf(title_data[1], "SCU_EmergencyStopReq : Ok");

    sprintf(title_data[2], "SCU_BrakeReq : %.1f", SCU_BrakeReq);

    sprintf(title_data[3], "SCU_AutoTrqWhlReq : %.1f", SCU_AutoTrqWhlReq);

    switch(SCU_DrvModeReq) {
        case 0:
            sprintf(title_data[4], "SCU_DrvModeReq : Reserved");
            break;
        case 1:
            sprintf(title_data[4], "SCU_DrvModeReq : Manual");
            break;
        case 2:
            sprintf(title_data[4], "SCU_DrvModeReq : Remote");
            break;
        case 3:
            sprintf(title_data[4], "SCU_DrvModeReq : Auto");
            break;
        default:
            sprintf(title_data[4], "SCU_DrvModeReq : ERROR");
            break;
    }


    //当前驾驶模式
    switch(vcu.VCU_DrvModeAct) {
        case 0:
            sprintf(title_data[5], "VCU_DrvModeAct : Reserved");
            break;
        case 1:
            sprintf(title_data[5], "VCU_DrvModeAct : Manual");
            break;
        case 2:
            sprintf(title_data[5], "VCU_DrvModeAct : Remote");
            break;
        case 3:
            sprintf(title_data[5], "VCU_DrvModeAct : Auto");
            break;
        default:
            sprintf(title_data[5], "VCU_DrvModeAct : error");
            break;
    }

    //档位信息
    switch(vcu.VCU_GearPos) {
        case 0:
            sprintf(title_data[6], "VCU_GearPos : Neutral");
            break;
        case 1:
            sprintf(title_data[6], "VCU_GearPos : Forward gear");
            break;
        case 2:
            sprintf(title_data[6], "VCU_GearPos : Backward gear");
            break;
        case 3:
            sprintf(title_data[6], "VCU_GearPos : P gear");
            break;
        default:
            sprintf(title_data[6], "VCU_GearPos : error");
            break;
    }


    //当前扭矩
    sprintf(title_data[7], "VCU_TrqWhlAct : %.1f", vcu.VCU_TrqWhlAct);

    //当前车速
    if(bcs.BCS_VehSpd > 240 || bcs.BCS_VehSpd < 0)
        sprintf(title_data[8], "BCS_VehSpd : error");
    else
        sprintf(title_data[8], "BCS_VehSpd : %.5f", bcs.BCS_VehSpd);


    //当前油门实际踏板位置
    sprintf(title_data[9], "VCU_EMS_AccPedalActPst : %.3f", vcu.VCU_EMS_AccPedalActPst);


    //当前制动踏板位置
    sprintf(title_data[10], "VCU_BrkPedPst : %.3f", vcu.VCU_BrkPedPst);



    //状态是否允许进入自动模式
    switch(vcu.VCU_CrntGearLvl) {
        case 0:
            sprintf(title_data[11], "VCU_CrntGearLvl : Invalid");
            break;
        case 1:
            sprintf(title_data[11], "VCU_CrntGearLvl : 'D' Drive gear");
            break;
        case 2:
            sprintf(title_data[11], "VCU_CrntGearLvl : 'N' Neutral gear'");
            break;
        case 3:
            sprintf(title_data[11], "VCU_CrntGearLvl : 'R' Reverse gear");
            break;
        case 4:
            sprintf(title_data[11], "VCU_CrntGearLvl : 'P' Reverse gear");
            break;
        default:
            sprintf(title_data[11], "VCU_CrntGearLvl : Not used");
            break;
    }

    switch(elseid.BCM_KeySt) {
        case 0:
            sprintf(title_data[12], "BCM_KeySt : Off");
            break;
        case 1:
            sprintf(title_data[12], "BCM_KeySt : Acc");
            break;
        case 2:
            sprintf(title_data[12], "BCM_KeySt : On");
            break;
        case 3:
            sprintf(title_data[12], "BCM_KeySt : Crank");
            break;
        default:
            sprintf(title_data[12], "BCM_KeySt : ERROR");
            break;
    }

    if(vcu.VCU_CrntGearLvl == 2 && elseid.BCM_KeySt == 2)
        sprintf(title_data[13], "Allow to enter : YES");
    else
        sprintf(title_data[13], "**Allow to enter : NO**");

    for(int i = 0; i<5; i++)
        putText(Display_control, title_data[i], Point(2400, 800+50*i), fontFace, fontScale,CV_RGB(0,0,255) , 2*thickness);
    for(int i = 5; i<11; i++)
        putText(Display_control, title_data[i], Point(2400, 800+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);

    for(int i = 11; i<14; i++)
        putText(Display_control, title_data[i], Point(2400, 800+50*i), fontFace, fontScale,CV_RGB(255,0,255) , 2*thickness);
}

