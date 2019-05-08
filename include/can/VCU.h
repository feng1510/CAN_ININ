#ifndef VCU_H
#define VCU_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>
#include "define.h"

using namespace std;
using namespace cv;

class VCU
{
public:
    VCU();
    virtual ~VCU();

    /*********************** VCU_2_P 0x360 **********************/
    __u8 VCU_ElcSysFault;
    __u8 VCU_ElcSysErr;
    float VCU_BMS_BattSOC;             //- resolution:0.3937% per bit    - offset:0
    __u8 VCU_DerateLampSt;             //- resolution: 1km per bit    - offset: 0

    /*********************** VCU_7_P 0x39F **********************/
    unsigned short int VCU_VehRng;

    /********************** VCU_9_P 0x2AB ************************************/
    bool VCU_BrkPedPstVD;             
    __u8 VCU_CrntGearLvl;
    float VCU_BrkPedPst;              //- resolution: 0.392% per bit  - offset: 0

    /********************** VCU_18_P 0x37C ************************************/
    __u8 VCU_GearPos;
    __u8 VCU_EPBMode;
    bool VCU_bVehEmergencyStopReq;
    bool VCU_bVehSoftStopReq;
    __u8 VCU_DrvModeAct;
    float VCU_TrqWhlAct;            //-resolution: 1.5 Nm per bit   -offset: -3000
    bool VCU_BrkSysFault;

    /********************** VCU_19_P 0x37D **********************************/
    float VCU_TqGndNegMax;        //-resolution: 1.5 Nm per bit  -offset: -3000
    float VCU_TqGndPosMax;        //-resolution: 1.5 Nm per bit  -offset: -3000

    /********************* VCU_20 0x186 *******************************/
    float VCU_ACCTargetAccel;
    bool VCU_BrakePreferredReq;
    __u8 VCU_20_P_MsgCounter;
    __u8 VCU_ACCModeForESP;
    __u8 VCU_20_P_Checksum;

    /******************** VCU_21 0x187 ********************************/
    float VCU_AEBTargetDecel;
    bool VCU_AEBDecelCtrlReq;
    __u8 VCU_21_P_MsgCounter;
    __u8 VCU_21_P_Checksum;

    /******************** VCU_EMS_1 0x18D *****************************/
    bool VCU_EMS_BrkPedalStVD;
    bool VCU_EMS_BrkPedalSt;

    /******************** VCU_EMS_6 0x279 ****************************/
    float VCU_EMS_AccPedalActPst;

    /********************* decode ********************************/
    void VCU_2_P_decode(struct can_frame &frame);
    void VCU_7_P_decode(struct can_frame &frame);
    void VCU_9_P_decode(struct can_frame &frame);
    void VCU_18_P_decode(struct can_frame &frame);
    void VCU_19_P_decode(struct can_frame &frame);
    void VCU_20_decode(struct can_frame &frame);
    void VCU_21_decode(struct can_frame &frame);
    void VCU_EMS_1_decode(struct can_frame &frame);
    void VCU_EMS_6_decode(struct can_frame &frame);


    /***************** diaplay ******************************/
    void VCU_display(cv::Mat Display);



private:
    /* data */
};







#endif