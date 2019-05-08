#include <iostream>
#include "can/EPS.h"

EPS::EPS() {

    cout << "EPS::init" << endl;

    /**************************   EPS_1   0x366  ***************************/
    EPS_StrngWhlTorqVD = false;
    EPS_WarnLamp = false;
    //EPS_SteringAngle
    EPS_SteeringAngle_invalid_flag = false;
    EPS_SteeringAngle_resolution = 0.1;
    EPS_SteeringAngle_offset = -780;
    EPS_SteeringAngle_min = -780;
    EPS_SteeringAngle_max = 779.9;
    EPS_SteeringAngle_invalid_value = 0xFFFF;

    //EPS_SteeringAngleSpt
    EPS_SteeringAngleSpt_resoultion = 4;
    EPS_SteeringAngleSpt_offset = 0;
    EPS_SteeringAngleSpt_min = 0;
    EPS_SteeringAngleSpt_max = 1016;
    EPS_SteeringAngleSpt_invalid_value = 0xFF;

    //EPS_StrngWhlTorq
    EPS_StrngWhlTorq_invalid_flag = false;
    EPS_StrngWhlTorq_resolution = 0.1;
    EPS_StrngWhlTorq_offset = -12.7;
    EPS_StrngWhlTorq_min = -12.5;
    EPS_StrngWhlTorq_max = 12.5;
    EPS_StrngWhlTorq_invalid_value = 0xFF;

    /*****************************  EPS_11  0x365  *******************************************/
    EPS_ControlMode = 1;
    EPS_FailureSt = false;
    /****************************  EPB_1 0x397 ************************************/
    EPB_SysSt = 0;

}

//EPS_1的解码，其中包括EPS_StrngWhlTorqVD、EPS_WarnLamp、EPS_SteeringAngle
void EPS::EPS_1_decode(struct can_frame &EPS_frame) {
    __u8 EPS_StrngWhlTorqVD_decode;
    __u16 EPS_SteeringAngle_decode;
    
    cout << "------EPS_1_decode------" << endl;

    //EPS_WarnLamp
    EPS_WarnLamp = (EPS_frame.data[0] >> 1) & 0x01;

    //EPS_SteeringAngle
    EPS_SteeringAngle_decode = EPS_frame.data[1];
    EPS_SteeringAngle_decode = (EPS_SteeringAngle_decode << 8)|EPS_frame.data[2];
    if(EPS_SteeringAngle_decode == EPS_SteeringAngle_invalid_value) {
        EPS_SteeringAngle_invalid_flag = true;
        cout << "***Waring : EPS_SteeringAngle_invalid!***" <<endl;
    }
        
    else {
        EPS_SteeringAngle_invalid_flag = false;
        EPS_SteeringAngle = EPS_SteeringAngle_decode * EPS_SteeringAngle_resolution + EPS_SteeringAngle_offset;
        cout << "***EPS_SteeringAngle : " << EPS_SteeringAngle << endl;
    }

    //EPS_StrngWhlTorqVD
    EPS_StrngWhlTorqVD = EPS_frame.data[0] & 0x01;

    if(EPS_StrngWhlTorqVD)
        cout << "EPS_StrngWhlTorqVD : Valid" << endl;
    else
        cout << "EPS_StrngWhlTorqVD : Not Valid" << endl;

    //EPS_StrngWhlTorq

    EPS_SteeringAngle_decode = EPS_frame.data[5];
    if(EPS_SteeringAngle_decode == EPS_StrngWhlTorq_invalid_value)
        EPS_StrngWhlTorq_invalid_flag = true;
    else {
        EPS_StrngWhlTorq_invalid_flag = false;
        EPS_StrngWhlTorq = EPS_SteeringAngle_decode*EPS_StrngWhlTorq_resolution+EPS_StrngWhlTorq_offset;
    }
    
}


void EPS::EPS_11_decode(struct can_frame &EPS_11_frame) {
    EPS_ControlMode = EPS_11_frame.data[1] & 0x03; //Byte1的低两位
    EPS_FailureSt = (EPS_11_frame.data[1] >> 2) & 0x01;
}


void EPS::EPB_decode(struct can_frame frame) {
    EPB_SysSt = frame.data[1] & 0x07;
}


void EPS::EPS_display(cv::Mat Display) {
    
    char title_data[40];
    if(EPS_SteeringAngle_invalid_flag || EPS_SteeringAngle < EPS_SteeringAngle_min || EPS_SteeringAngle > EPS_SteeringAngle_max)
        sprintf(title_data,"**Waring : EPS_SteeringAngle_invalid!**");
    else
        sprintf(title_data,"EPS_SteeringAngle: %.1f",EPS_SteeringAngle);

    char title_data2[40];
    if(EPS_WarnLamp)
        sprintf(title_data2, "EPS_WarnLamp :  Fail present");
    else
        sprintf(title_data2, "EPS_WarnLamp :  Fail not present");

    char title_data3[40];
    if(EPS_StrngWhlTorq_invalid_flag || EPS_StrngWhlTorq < EPS_StrngWhlTorq_min || EPS_StrngWhlTorq > EPS_StrngWhlTorq_max)
        sprintf(title_data3, "**Waring : EPS_StrngWhlPorq_invalid!**");
    else
        sprintf(title_data3, "EPS_StrngWhlPorq : %.1f", EPS_StrngWhlTorq);

    //putText(Display, title_data, Point(50,100), fontFace, fontScale,CV_RGB(255,255,255) , 3*thickness);
    putText(Display, title_data, Point(50,50), fontFace, fontScale,CV_RGB(255,0,0) , 2*thickness);
    putText(Display, title_data2, Point(50,200), fontFace, fontScale,CV_RGB(255,0,0) , 2*thickness);
    putText(Display, title_data3, Point(50,250), fontFace, fontScale,CV_RGB(255,0,0) , 2*thickness);

    
    char title_data4[40];
    char title_data5[40];
    switch(EPS_ControlMode) {
        case 0:
            sprintf(title_data4,"EPS_ControlMode : Idle");
            break;
        case 1:
            sprintf(title_data4,"EPS_ControlMode : Manual Assist");
            break;
        case 2:
            sprintf(title_data4,"EPS_ControlMode : Auto Steering");
            break;
        default:
            sprintf(title_data4,"**ERROR:EPS_ControlMode : decode error**");
            break;
    }

    if(EPS_FailureSt) 
        sprintf(title_data5,"EPS_FailureSt : fail");
    else
        sprintf(title_data5,"EPS_FailureSt : not fail");

    putText(Display, title_data4, Point(50,100), fontFace, fontScale,CV_RGB(255,0,0) , 2*thickness);
    putText(Display, title_data5, Point(50,150), fontFace, fontScale,CV_RGB(255,0,0) , 2*thickness);

    //EPB_1
    char title_data6[50];
    switch(EPB_SysSt) {
        case 0:
            sprintf(title_data6,"EPB_SysSt : Released");
            break;
        case 1:
            sprintf(title_data6,"EPB_SysSt : Applied");
            break;
        case 2:
            sprintf(title_data6,"EPB_SysSt : Releasing");
            break;
        case 3:
            sprintf(title_data6,"EPB_SysSt : Fault (Stop and not sure state)");
            break;
        case 4:
            sprintf(title_data6,"EPB_SysSt : Applying");
            break;
        case 5:
            sprintf(title_data6,"EPB_SysSt : Disengaged");
            break;
        case 6:
            sprintf(title_data6,"EPB_SysSt : Not used");
            break;
        case 7:
            sprintf(title_data6,"EPB_SysSt : Not used");
            break;
        default:
            break;
    }

    putText(Display, title_data6, Point(700, 50), fontFace, fontScale,CV_RGB(255,0,0) , 2*thickness);

}


