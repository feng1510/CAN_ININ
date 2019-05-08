#include <iostream>
#include "can/ELSE.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <linux/can.h>

using namespace std;
using namespace cv;

ELSE::ELSE() {

    GSM_GearShiftLeverPstReq = 0;

    SAS_SteeringAngleVD = false;
    SAS_SteeringAngleSpd = 0;
    SAS_SteeringAngle = 0;
    SAS_CalibratedSt = false;

    BCM_KeySt = 0;
}


ELSE::~ELSE() {
    cout << "~~ELSE()" << endl;
}



void ELSE::GSM_1_decode(struct can_frame frame) {
    GSM_GearShiftLeverPstReq = frame.data[1] & 0x0F;
}

void ELSE::SAS_1_decode(struct can_frame frame) {
    __u16 decode1 = 0x0000;
    __u16 decode2 = 0x0000;

    SAS_SteeringAngleVD = frame.data[1] & 0x01;

    SAS_SteeringAngleSpd = frame.data[2] * 4;      //-resolution: 4deg/sec per bit   -offset:0

    decode1 = frame.data[3];
    decode2 = frame.data[4];
    decode2 = (decode1 << 8) | decode2;
    SAS_SteeringAngle = decode2 * 0.1 - 780;       //-resolution: 0.1 deg per bit     -offset:-780

    SAS_CalibratedSt = (frame.data[1] >> 1) & 0x01;
}

void ELSE::GW_BCM_2_P_decode(struct can_frame frame) {
    BCM_KeySt = frame.data[1] & 0x03;
}


/************************************************/
void ELSE::ELSE_display(cv::Mat Display) {
    char title_data[6][50];

    switch(GSM_GearShiftLeverPstReq) {
        case 0:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : Invalid");
            break;
        case 1:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : P");
            break;
        case 2:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : R");
            break;
        case 3:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : N");
            break;
        case 4:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : D");
            break;
        case 5:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : D+");
            break;
        case 6:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : D-");
            break;
        case 7:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : S");
            break;
        case 8:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : M");
            break;
        case 9:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : M+");
            break;
        case 10:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : M-");
            break;
        case 11:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : Default");
            break;
        default:
            sprintf(title_data[0], "GSM_GearShiftLeverPstReq : Not used");
            break;
    }

    if(SAS_SteeringAngleVD)
        sprintf(title_data[1], "SAS_SteeringAngleVD : Valid");
    else
        sprintf(title_data[1], "SAS_SteeringAngleVD : Not Valid");

    sprintf(title_data[2], "SAS_SteeringAngleSpd : %d", SAS_SteeringAngleSpd);

    sprintf(title_data[3], "SAS_SteeringAngle : %.1f", SAS_SteeringAngle);

    if(SAS_CalibratedSt)
        sprintf(title_data[4], "SAS_CalibratedSt : Calibrated");
    else
        sprintf(title_data[4], "SAS_CalibratedSt : Not calibrated");

    
    switch(BCM_KeySt) {
        case 0:
            sprintf(title_data[5], "BCM_KeySt : Off");
            break;
        case 1:
            sprintf(title_data[5], "BCM_KeySt : Acc");
            break;
        case 2:
            sprintf(title_data[5], "BCM_KeySt : On");
            break;
        case 3:
            sprintf(title_data[5], "BCM_KeySt : Crank");
            break;
        
        default:
            break;
    }


    for (int i = 0; i<6; i++) {
        putText(Display, title_data[i], Point(2400, 50+50*i), fontFace, fontScale,CV_RGB(0,0,0) , 2*thickness);
    }


}