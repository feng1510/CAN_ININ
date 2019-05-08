#include <iostream>
#include "can/VCU.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>

using namespace std;
using namespace cv;

VCU::VCU() {
    /*********************** VCU_2_P 0x360 **********************/
    VCU_ElcSysFault = 0;
    VCU_ElcSysErr = 0;
    VCU_BMS_BattSOC = 0;             //- resolution:0.3937% per bit    - offset:0
    VCU_DerateLampSt = 0;             //- resolution: 1km per bit    - offset: 0

    /*********************** VCU_7_P 0x39F **********************/
    VCU_VehRng = 0x3E7;        //invalid value 0x3E8

    /********************** VCU_9_P 0x2AB ************************************/
    VCU_BrkPedPstVD = false;             
    VCU_CrntGearLvl = 0;
    VCU_BrkPedPst = 0;              //- resolution: 0.392% per bit  - offset: 0

    /********************** VCU_18_P 0x37C ************************************/
    VCU_GearPos = 0;
    VCU_EPBMode = 0;
    VCU_bVehEmergencyStopReq = false;
    VCU_bVehSoftStopReq = false;
    VCU_DrvModeAct = 0;
    VCU_TrqWhlAct = 0;            //-resolution: 1.5 Nm per bit   -offset: -3000
    VCU_BrkSysFault = false;

    /********************** VCU_19_P 0x37D **********************************/
    VCU_TqGndNegMax = 0;        //-resolution: 1.5 Nm per bit  -offset: -3000
    VCU_TqGndPosMax = 0;        //-resolution: 1.5 Nm per bit  -offset: -3000

    /********************* VCU_20 0x186 *******************************/
    VCU_ACCTargetAccel = 0;
    VCU_BrakePreferredReq = false;
    VCU_20_P_MsgCounter = 0;
    VCU_ACCModeForESP = 0;
    VCU_20_P_Checksum = 0;

    /******************** VCU_21 0x187 ********************************/
    VCU_AEBTargetDecel = 0;
    VCU_AEBDecelCtrlReq = false;
    VCU_21_P_MsgCounter = 0;
    VCU_21_P_Checksum = 0;

    /******************** VCU_EMS_1 0x18D *****************************/
    VCU_EMS_BrkPedalStVD = false;
    VCU_EMS_BrkPedalSt = false;

    /******************** VCU_EMS_6 0x279 ****************************/
    VCU_EMS_AccPedalActPst = 0;
}
VCU::~VCU() {
    cout << "~VCU()" << endl;
}


   /*********************** decode **************************/
void VCU::VCU_2_P_decode(struct can_frame &frame) {
    __u8 decode;
    VCU_ElcSysFault = frame.data[0] & 0x0F;
    VCU_ElcSysErr = (frame.data[1] >> 5) & 0x03;
    VCU_BMS_BattSOC = frame.data[3] * 0.3937;    //单位  %
    VCU_DerateLampSt = (frame.data[7] >> 5) & 0x03;
}

void VCU::VCU_7_P_decode(struct can_frame &frame) {
    __u16 decode1 = 0x0000;
    __u16 decode2 = 0x0000;
    
    decode1 = frame.data[2] & 0x03;
    decode2 = frame.data[3];
    decode2 = (decode1 << 8) | decode2;
    VCU_VehRng = decode2;    //- resolution: 1km per bit  - offset: 0  单位km
}

void VCU::VCU_9_P_decode(struct can_frame &frame) {
    VCU_BrkPedPstVD = (frame.data[3] >> 3) & 0x01;
    VCU_CrntGearLvl = (frame.data[4] >> 4) & 0x07;
    VCU_BrkPedPst = frame.data[6] *0.392;          //单位   %
}

void VCU::VCU_18_P_decode(struct can_frame &frame) {
    __u16 decode1 = 0x0000;
    __u16 decode2 = 0x0000;
    VCU_GearPos = (frame.data[0] >> 1) & 0x03;     //档位：0=Neutral   1=Forward gear    2=Backward gear
    VCU_EPBMode = (frame.data[0] >> 4) & 0x03;     //电子手刹：0=Neutral  1=Halt  2=Release
    VCU_bVehEmergencyStopReq = frame.data[6] & 0x01;
    VCU_bVehSoftStopReq = (frame.data[6] >> 1) & 0x01;
    VCU_DrvModeAct = (frame.data[6] >> 2) & 0x03;

    decode1 = (frame.data[5] >> 4) & 0x0F;
    decode2 = ((frame.data[5] << 4) & 0xF0) | ((frame.data[6] >> 4) & 0x0F);
    decode2 = (decode1 << 8) | decode2;
    VCU_TrqWhlAct = decode2 * 1.5 - 3000;   //-resolution: 1.5 Nm per bit  -offset: -3000
    
    VCU_BrkSysFault = (frame.data[7] >> 7) & 0x01;
}

void VCU::VCU_19_P_decode(struct can_frame &frame) {

    __u16 decode1 = 0x0000;
    __u16 decode2 = 0x0000;
    decode1 = (frame.data[5] >> 3) & 0x07;
    decode2 = ((frame.data[5] << 5) & 0xE0) | ((frame.data[6] >> 3) & 0x1F);
    decode2 = (decode1 << 8) | decode2;
    VCU_TqGndNegMax = decode2 * 1.5 - 3000;    //-resolution: 1.5 Nm per bit  -offset: -3000

    decode1 = frame.data[6] & 0x07;
    decode2 = frame.data[7];
    decode2 = (decode1 << 8) | decode2;
    VCU_TqGndPosMax = decode2 * 1.5 - 3000;
}   



void VCU::VCU_20_decode(struct can_frame &frame) {
    VCU_ACCTargetAccel = frame.data[0] * 0.05 - 5;       //-resolution:0.05m/s2 per bit  -offset:-5
    VCU_BrakePreferredReq = (frame.data[5] >> 1) & 0x01;
    VCU_20_P_MsgCounter = frame.data[6] & 0x0F;          //-resolution:1 per bit   -offset:0
    VCU_ACCModeForESP =  (frame.data[6] >> 4) & 0x07;
    VCU_20_P_Checksum = frame.data[7];
}


void VCU::VCU_21_decode(struct can_frame &frame) {
    
    __u16 decode1 = 0x0000;
    __u16 decode2 = 0x0000;
    decode1 = frame.data[0];
    decode2 = frame.data[1];
    decode2 = (decode1 << 8) | decode2;
    VCU_AEBTargetDecel = decode2 * 0.0004882 - 16;     //-resolution: 0.0004882m/s2 per bit  -offset:-16

    VCU_AEBDecelCtrlReq = frame.data[2] & 0x01;

    VCU_21_P_MsgCounter = frame.data[6] & 0x0F;

    VCU_21_P_Checksum = frame.data[7];
}

void VCU::VCU_EMS_1_decode(struct can_frame &frame) {
    VCU_EMS_BrkPedalStVD = (frame.data[5] >> 4) & 0x01;
    VCU_EMS_BrkPedalSt = (frame.data[5] >> 5) & 0x01;
}

void VCU::VCU_EMS_6_decode(struct can_frame &frame) {
    VCU_EMS_AccPedalActPst = frame.data[0] * 0.392;       //-resolution: 0.392% per bit  -offset:0
}



/************ dispaly ***************/
void VCU::VCU_display(cv::Mat Display) {
    char title_data[29][60];
   
   //VCU_2_P
    switch(VCU_ElcSysFault) {
        case 0:
            sprintf(title_data[0], "VCU_ElcSysFault : No error");
            break;
        case 1:
            sprintf(title_data[0], "VCU_ElcSysFault : Reduce Power");
            break;
        case 2:
            sprintf(title_data[0], "VCU_ElcSysFault : Reserved");
            break;
        case 3:
            sprintf(title_data[0], "VCU_ElcSysFault : Reserved");
            break;
        case 4:
            sprintf(title_data[0], "VCU_ElcSysFault : Reserved");
            break;
        case 5:
            sprintf(title_data[0], "VCU_ElcSysFault : System fault");
            break;
        case 6:
            sprintf(title_data[0], "VCU_ElcSysFault : Reserved");
            break;
        default:
            sprintf(title_data[0], "VCU_ElcSysFault : Not used");
            break;
    }

    //HEV系统错误警告指示灯
    switch(VCU_ElcSysErr) {
        case 0:
            sprintf(title_data[1], "VCU_ElcSysErr : Off");
            break;
        case 1:
            sprintf(title_data[1], "VCU_ElcSysErr : Lamp on");
            break;
        case 2:
            sprintf(title_data[1], "VCU_ElcSysErr : Lamp blink");
            break;
        case 3:
            sprintf(title_data[1], "VCU_ElcSysErr : Not used");
            break;
        default:
            break;
    }

    sprintf(title_data[2], "VCU_BMS_BattSOC : %.4f", VCU_BMS_BattSOC);

     switch(VCU_DerateLampSt) {
        case 0:
            sprintf(title_data[3], "VCU_DerateLampSt : Lamp off");
            break;
        case 1:
            sprintf(title_data[3], "VCU_DerateLampSt : Lamp on");
            break;
        case 2:
            sprintf(title_data[3], "VCU_DerateLampSt : Lamp blink");
            break;
        case 3:
            sprintf(title_data[3], "VCU_DerateLampSt : Not used");
            break;
        default:
            break;
    }

    //VCU_7_P
    sprintf(title_data[4], "VCU_VehRng : %d", VCU_VehRng);


    /****************************************************************/
    //VCU_9_P
    if(VCU_BrkPedPstVD)
        sprintf(title_data[5], "VCU_BrkPedPstVD : Valid");
    else
        sprintf(title_data[5], "VCU_BrkPedPstVD : Not valid");

    switch(VCU_CrntGearLvl) {
        case 0:
            sprintf(title_data[6], "VCU_CrntGearLvl : Invalid");
            break;
        case 1:
            sprintf(title_data[6], "VCU_CrntGearLvl : 'D' Drive gear");
            break;
        case 2:
            sprintf(title_data[6], "VCU_CrntGearLvl : 'N' Neutral gear'");
            break;
        case 3:
            sprintf(title_data[6], "VCU_CrntGearLvl : 'R' Reverse gear");
            break;
        case 4:
            sprintf(title_data[6], "VCU_CrntGearLvl : 'P' Reverse gear");
            break;
        default:
            sprintf(title_data[6], "VCU_CrntGearLvl : Not used");
            break;
    }

    sprintf(title_data[7], "VCU_BrkPedPst : %.3f", VCU_BrkPedPst);

/*********************************************************************/
//VCU_18_P
    switch(VCU_GearPos) {
        case 0:
            sprintf(title_data[8], "VCU_GearPos : Neutral");
            break;
        case 1:
            sprintf(title_data[8], "VCU_GearPos : Forward gear");
            break;
        case 2:
            sprintf(title_data[8], "VCU_GearPos : Backward gear");
            break;
        case 3: 
            sprintf(title_data[8], "VCU_GearPos : P gear");
            break;
        default:
            sprintf(title_data[8], "VCU_GearPos : error");
            break;
    }

    switch(VCU_EPBMode) {
        case 0:
            sprintf(title_data[9], "VCU_EPBMode : Neutral");
            break;
        case 1:
            sprintf(title_data[9], "VCU_EPBMode : Halt");
            break;
        case 2:
            sprintf(title_data[9], "VCU_EPBMode : Release");
            break;
        default:
            break;
    }

    if(VCU_bVehEmergencyStopReq)
        sprintf(title_data[10], "VCU_bVehEmergencyStopReq : Active");
    else
        sprintf(title_data[10], "VCU_bVehEmergencyStopReq : Not active");
    
    if(VCU_bVehSoftStopReq)
        sprintf(title_data[11], "VCU_bVehSoftStopReq : Active");
    else
        sprintf(title_data[11], "VCU_bVehSoftStopReq : Not active");

    switch(VCU_DrvModeAct) {
        case 0:
            sprintf(title_data[12], "VCU_DrvModeAct : Reserved");
            break;
        case 1:
            sprintf(title_data[12], "VCU_DrvModeAct : Manual");
            break;
        case 2:
            sprintf(title_data[12], "VCU_DrvModeAct : Remote");
            break;
        case 3:
            sprintf(title_data[12], "VCU_DrvModeAct : Auto");
            break;
        default:
            sprintf(title_data[12], "VCU_DrvModeAct : error");
            break;
    }

    sprintf(title_data[13], "VCU_TrqWhlAct : %.1f", VCU_TrqWhlAct);

    if(VCU_BrkSysFault)
        sprintf(title_data[14], "VCU_BrkSysFault : Fault");
    else
        sprintf(title_data[14], "VCU_BrkSysFault : Not Fault");

    /*************************************************************/
    //VCU_19_P
    sprintf(title_data[15], "VCU_TqGndNegMax : %.1f", VCU_TqGndNegMax);
    sprintf(title_data[16], "VCU_TqGndPosMax : %.1f", VCU_TqGndPosMax);

    /*******************************************************/
    //VCU_20
    sprintf(title_data[17], "VCU_ACCTargetAccel : %.2f", VCU_ACCTargetAccel);

    if(VCU_BrakePreferredReq)
        sprintf(title_data[18], "VCU_BrakePreferredReq : Request");
    else
        sprintf(title_data[18], "VCU_BrakePreferredReq : Not request");
    
    sprintf(title_data[19], "VCU_20_P_MsgCounter : %d", VCU_20_P_MsgCounter);

    switch(VCU_ACCModeForESP) {
        case 0:
            sprintf(title_data[20], "VCU_ACCModeForESP : Off mode");
            break;
        case 1:
            sprintf(title_data[20], "VCU_ACCModeForESP : Passive mode");
            break;
        case 2:
            sprintf(title_data[20], "VCU_ACCModeForESP : Standby mode");
            break;
        case 3:
            sprintf(title_data[20], "VCU_ACCModeForESP : Active control mode");
            break;
        case 4:
            sprintf(title_data[20], "VCU_ACCModeForESP : Brake only mode");
            break;
        case 5:
            sprintf(title_data[20], "VCU_ACCModeForESP : Override");
            break;
        case 6:
            sprintf(title_data[20], "VCU_ACCModeForESP : Standstill");
            break;
        case 7:
            sprintf(title_data[20], "VCU_ACCModeForESP : Failure mode");
            break;
        default:
            sprintf(title_data[20], "VCU_ACCModeForESP : error");
            break;
    }

    sprintf(title_data[21], "VCU_20_P_Checksum : %d", VCU_20_P_Checksum);

/****************************************************************/
    //VCU_21
    sprintf(title_data[22], "VCU_AEBTargetDecel : %.7f", VCU_AEBTargetDecel);
    
    if(VCU_AEBDecelCtrlReq)
        sprintf(title_data[23], "VCU_AEBDecelCtrlReq : Request");
    else
        sprintf(title_data[23], "VCU_AEBDecelCtrlReq : Not Request");

    sprintf(title_data[24], "VCU_21_P_MsgCounter : %d", VCU_21_P_MsgCounter);

    sprintf(title_data[25], "VCU_21_P_Checksum : %d", VCU_21_P_Checksum);

/**************************************************************/
    //VCU_EMS_1
    if(VCU_EMS_BrkPedalStVD)
        sprintf(title_data[26], "VCU_EMS_BrkPedalStVD : Valid");
    else
        sprintf(title_data[26], "VCU_EMS_BrkPedalStVD : Not valid");

    
    if(VCU_EMS_BrkPedalSt)
        sprintf(title_data[27], "VCU_EMS_BrkPedalSt : Pressed");
    else
        sprintf(title_data[27], "VCU_EMS_BrkPedalSt : Not Pressed");

    sprintf(title_data[28], "VCU_EMS_AccPedalActPst : %.3f", VCU_EMS_AccPedalActPst);


    for (int i = 0; i <=7; i++) {
        putText(Display, title_data[i], Point(1600, 50+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }

    for (int i = 8; i<=14; i++) {
        putText(Display, title_data[i], Point(1600, 100+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }

    for (int i = 15; i<=21; i++) {
        putText(Display, title_data[i], Point(1600, 150+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }

    for (int i = 22; i<=28; i++) {
        putText(Display, title_data[i], Point(1600, 200+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }

}
